#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <common/mavlink.h>

namespace
{

constexpr uint8_t kMavDataStreamRawSensors = 1;
constexpr uint8_t kMavDataStreamExtra1 = 10;
constexpr uint8_t kMavModeFlagCustomModeEnabled = 1;
constexpr uint8_t kAttitudeTargetTypeMaskIgnoreAttitude = 128;
constexpr uint16_t kMavCmdComponentArmDisarm = 400;
constexpr uint8_t kMavTypeOnboardController = 18;
constexpr uint8_t kMavAutopilotInvalid = 8;
constexpr uint8_t kMavStateActive = 4;
constexpr uint8_t kMavCompIdOnboardComputer = 191;

speed_t to_baud_constant(int baudrate)
{
  switch (baudrate) {
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

double pressure_to_altitude_m(double pressure_hpa)
{
  constexpr double sea_level_pressure_hpa = 1013.25;
  constexpr double exponent = 0.190284;

  if (pressure_hpa <= 1e-6) {
    return 0.0;
  }

  return 44330.0 * (1.0 - std::pow(pressure_hpa / sea_level_pressure_hpa, exponent));
}

}  // namespace

class FcBridge
{
public:
  virtual ~FcBridge() = default;

  virtual bool connect() = 0;
  virtual void enter_offboard_mode() = 0;
  virtual void send_body_rate_and_thrust(
    const geometry_msgs::msg::Vector3Stamped & body_rate,
    float thrust) = 0;
  virtual void arm() = 0;
  virtual void disarm() = 0;
  virtual void stop_takeoff() = 0;
  virtual void poll() = 0;
  virtual bool take_attitude(geometry_msgs::msg::Vector3Stamped & attitude_msg) = 0;
  virtual bool take_baro_alt(std_msgs::msg::Float32 & baro_alt_msg) = 0;
};

class MavlinkBridgeBase : public FcBridge
{
public:
  MavlinkBridgeBase(
    rclcpp::Logger logger,
    uint8_t local_system_id,
    uint8_t local_component_id,
    uint8_t target_system_id,
    uint8_t target_component_id)
  : logger_(logger),
    local_system_id_(local_system_id),
    local_component_id_(local_component_id),
    target_system_id_(target_system_id),
    target_component_id_(target_component_id)
  {
  }

  void enter_offboard_mode() override
  {
    if (!is_connected()) {
      return;
    }

    mavlink_message_t msg {};
    constexpr uint8_t px4_custom_main_mode_offboard = 6;

    mavlink_msg_set_mode_pack(
      local_system_id_,
      local_component_id_,
      &msg,
      target_system_id_,
      kMavModeFlagCustomModeEnabled,
      px4_custom_main_mode_offboard << 16);

    write_message(msg);
  }

  void send_body_rate_and_thrust(
    const geometry_msgs::msg::Vector3Stamped & body_rate,
    float thrust) override
  {
    if (!is_connected()) {
      return;
    }

    mavlink_message_t msg {};
    const std::array<float, 4> q{1.0F, 0.0F, 0.0F, 0.0F};
    const std::array<float, 3> thrust_body{0.0F, 0.0F, 0.0F};
    const uint8_t type_mask = kAttitudeTargetTypeMaskIgnoreAttitude;

    const float clamped_thrust = std::clamp(thrust, 0.0F, 1.0F);

    mavlink_msg_set_attitude_target_pack(
      local_system_id_,
      local_component_id_,
      &msg,
      time_boot_ms(),
      target_system_id_,
      target_component_id_,
      type_mask,
      q.data(),
      static_cast<float>(body_rate.vector.x),
      static_cast<float>(body_rate.vector.y),
      static_cast<float>(body_rate.vector.z),
      clamped_thrust,
      thrust_body.data());

    write_message(msg);
  }

  void arm() override
  {
    enter_offboard_mode();
    send_command_long(kMavCmdComponentArmDisarm, 1.0F);
  }

  void disarm() override
  {
    constexpr float kForceDisarmMagic = 21196.0F;
    send_command_long(kMavCmdComponentArmDisarm, 0.0F, kForceDisarmMagic);
  }

  void stop_takeoff() override
  {
    // The legacy meaning of "takeoff_stop" is unclear in the recovered code.
    // Keep this as a safe no-op until the original vehicle-mode behavior is restored.
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000,
      "takeoff_stop received, but its legacy MAVLink mapping is unknown. No command sent.");
  }

  void poll() override
  {
    if (!is_connected()) {
      return;
    }

    send_heartbeat_if_needed();

    uint8_t buffer[512];
    while (true) {
      const ssize_t bytes_read = read_bytes(buffer, sizeof(buffer));
      if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        log_read_error();
        break;
      }

      if (bytes_read == 0) {
        break;
      }

      for (ssize_t i = 0; i < bytes_read; ++i) {
        mavlink_message_t msg {};
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &parse_status_)) {
          handle_message(msg);
        }
      }
    }
  }

  bool take_attitude(geometry_msgs::msg::Vector3Stamped & attitude_msg) override
  {
    if (!has_new_attitude_) {
      return false;
    }

    attitude_msg = last_attitude_msg_;
    has_new_attitude_ = false;
    return true;
  }

  bool take_baro_alt(std_msgs::msg::Float32 & baro_alt_msg) override
  {
    if (!has_new_baro_alt_) {
      return false;
    }

    baro_alt_msg = last_baro_alt_msg_;
    has_new_baro_alt_ = false;
    return true;
  }

  void set_clock(const rclcpp::Clock::SharedPtr & clock)
  {
    clock_ = clock;
  }

protected:
  virtual bool is_connected() const = 0;
  virtual ssize_t read_bytes(uint8_t * buffer, size_t length) = 0;
  virtual ssize_t write_bytes(const uint8_t * buffer, size_t length) = 0;
  virtual void log_read_error() = 0;
  virtual void log_write_error() = 0;

  void initialize_connection()
  {
    send_heartbeat();
    request_stream(kMavDataStreamExtra1, 30);
    request_stream(kMavDataStreamRawSensors, 20);
  }

private:
  uint32_t time_boot_ms() const
  {
    using namespace std::chrono;
    const auto now = steady_clock::now();
    return static_cast<uint32_t>(
      duration_cast<milliseconds>(now - started_at_).count());
  }

  void send_heartbeat_if_needed()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_heartbeat_sent_ >= std::chrono::seconds(1)) {
      send_heartbeat();
    }
  }

  void send_heartbeat()
  {
    mavlink_message_t msg {};
    mavlink_msg_heartbeat_pack(
      local_system_id_,
      local_component_id_,
      &msg,
      kMavTypeOnboardController,
      kMavAutopilotInvalid,
      0,
      0,
      kMavStateActive);
    write_message(msg);
    last_heartbeat_sent_ = std::chrono::steady_clock::now();
  }

  void request_stream(uint8_t stream_id, uint16_t message_rate_hz)
  {
    mavlink_message_t msg {};
    mavlink_msg_request_data_stream_pack(
      local_system_id_,
      local_component_id_,
      &msg,
      target_system_id_,
      target_component_id_,
      stream_id,
      message_rate_hz,
      1);
    write_message(msg);
  }

  void send_command_long(
    uint16_t command,
    float param1 = 0.0F,
    float param2 = 0.0F,
    float param3 = 0.0F,
    float param4 = 0.0F,
    float param5 = 0.0F,
    float param6 = 0.0F,
    float param7 = 0.0F)
  {
    if (!is_connected()) {
      return;
    }

    mavlink_message_t msg {};
    mavlink_msg_command_long_pack(
      local_system_id_,
      local_component_id_,
      &msg,
      target_system_id_,
      target_component_id_,
      command,
      0,
      param1,
      param2,
      param3,
      param4,
      param5,
      param6,
      param7);
    write_message(msg);
  }

  void write_message(const mavlink_message_t & msg)
  {
    if (!is_connected()) {
      return;
    }

    uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
    const ssize_t bytes_written = write_bytes(tx_buffer, len);
    if (bytes_written < 0) {
      log_write_error();
    }
  }

  void handle_message(const mavlink_message_t & msg)
  {
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:
      {
        if (target_system_id_ == 0) {
          target_system_id_ = msg.sysid;
        }
        if (target_component_id_ == 0) {
          target_component_id_ = msg.compid;
        }
        break;
      }
      case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t mav_attitude {};
        mavlink_msg_attitude_decode(&msg, &mav_attitude);

        geometry_msgs::msg::Vector3Stamped attitude_msg;
        attitude_msg.header.stamp = clock_->now();
        attitude_msg.vector.x = static_cast<double>(mav_attitude.roll);
        attitude_msg.vector.y = static_cast<double>(mav_attitude.pitch);
        attitude_msg.vector.z = static_cast<double>(mav_attitude.yaw);

        last_attitude_msg_ = attitude_msg;
        has_new_attitude_ = true;
        break;
      }
      case MAVLINK_MSG_ID_VFR_HUD:
      {
        mavlink_vfr_hud_t hud {};
        mavlink_msg_vfr_hud_decode(&msg, &hud);

        std_msgs::msg::Float32 baro_alt_msg;
        baro_alt_msg.data = hud.alt;
        last_baro_alt_msg_ = baro_alt_msg;
        has_new_baro_alt_ = true;
        break;
      }
      case MAVLINK_MSG_ID_SCALED_PRESSURE:
      {
        if (has_new_baro_alt_) {
          break;
        }

        mavlink_scaled_pressure_t pressure {};
        mavlink_msg_scaled_pressure_decode(&msg, &pressure);

        std_msgs::msg::Float32 baro_alt_msg;
        baro_alt_msg.data = static_cast<float>(pressure_to_altitude_m(pressure.press_abs));
        last_baro_alt_msg_ = baro_alt_msg;
        has_new_baro_alt_ = true;
        break;
      }
      default:
        break;
    }
  }

protected:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

private:
  uint8_t local_system_id_{42};
  uint8_t local_component_id_{kMavCompIdOnboardComputer};
  uint8_t target_system_id_{1};
  uint8_t target_component_id_{1};

  mavlink_status_t parse_status_{};

  geometry_msgs::msg::Vector3Stamped last_attitude_msg_{};
  std_msgs::msg::Float32 last_baro_alt_msg_{};
  bool has_new_attitude_{false};
  bool has_new_baro_alt_{false};

  std::chrono::steady_clock::time_point started_at_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_heartbeat_sent_{std::chrono::steady_clock::time_point::min()};
};

class RawSerialMavlinkBridge : public MavlinkBridgeBase
{
public:
  RawSerialMavlinkBridge(
    rclcpp::Logger logger,
    const std::string & serial_device,
    int baudrate,
    uint8_t local_system_id,
    uint8_t local_component_id,
    uint8_t target_system_id,
    uint8_t target_component_id)
  : MavlinkBridgeBase(
      logger, local_system_id, local_component_id, target_system_id, target_component_id),
    serial_device_(serial_device),
    baudrate_(baudrate)
  {
  }

  ~RawSerialMavlinkBridge() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  bool connect() override
  {
    fd_ = open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(
        logger_,
        "failed to open serial device %s: %s",
        serial_device_.c_str(),
        std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(logger_, "tcgetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    cfmakeraw(&tty);
    const speed_t baud = to_baud_constant(baudrate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(logger_, "tcsetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    initialize_connection();

    RCLCPP_INFO(
      logger_,
      "connected to FC over serial %s at %d baud",
      serial_device_.c_str(),
      baudrate_);
    return true;
  }

protected:
  bool is_connected() const override
  {
    return fd_ >= 0;
  }

  ssize_t read_bytes(uint8_t * buffer, size_t length) override
  {
    return read(fd_, buffer, length);
  }

  ssize_t write_bytes(const uint8_t * buffer, size_t length) override
  {
    return write(fd_, buffer, length);
  }

  void log_read_error() override
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000,
      "serial read failed: %s", std::strerror(errno));
  }

  void log_write_error() override
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000,
      "serial write failed: %s", std::strerror(errno));
  }

private:
  std::string serial_device_;
  int baudrate_{115200};
  int fd_{-1};
};

class UdpMavlinkBridge : public MavlinkBridgeBase
{
public:
  UdpMavlinkBridge(
    rclcpp::Logger logger,
    const std::string & bind_host,
    int bind_port,
    const std::string & remote_host,
    int remote_port,
    uint8_t local_system_id,
    uint8_t local_component_id,
    uint8_t target_system_id,
    uint8_t target_component_id)
  : MavlinkBridgeBase(
      logger, local_system_id, local_component_id, target_system_id, target_component_id),
    bind_host_(bind_host),
    bind_port_(bind_port),
    remote_host_(remote_host),
    remote_port_(remote_port)
  {
  }

  ~UdpMavlinkBridge() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

  bool connect() override
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(logger_, "failed to create UDP socket: %s", std::strerror(errno));
      return false;
    }

    sockaddr_in local_addr {};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(static_cast<uint16_t>(bind_port_));
    if (inet_pton(AF_INET, bind_host_.c_str(), &local_addr.sin_addr) != 1) {
      RCLCPP_ERROR(logger_, "invalid UDP bind_host: %s", bind_host_.c_str());
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) != 0) {
      RCLCPP_ERROR(
        logger_,
        "failed to bind UDP socket %s:%d: %s",
        bind_host_.c_str(),
        bind_port_,
        std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    const int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) != 0) {
      RCLCPP_ERROR(logger_, "failed to configure UDP socket non-blocking mode: %s", std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(static_cast<uint16_t>(remote_port_));
    if (inet_pton(AF_INET, remote_host_.c_str(), &remote_addr_.sin_addr) != 1) {
      RCLCPP_ERROR(logger_, "invalid UDP remote_host: %s", remote_host_.c_str());
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    initialize_connection();

    RCLCPP_INFO(
      logger_,
      "connected to FC over UDP %s:%d -> %s:%d",
      bind_host_.c_str(),
      bind_port_,
      remote_host_.c_str(),
      remote_port_);
    return true;
  }

protected:
  bool is_connected() const override
  {
    return socket_fd_ >= 0;
  }

  ssize_t read_bytes(uint8_t * buffer, size_t length) override
  {
    return recvfrom(socket_fd_, buffer, length, 0, nullptr, nullptr);
  }

  ssize_t write_bytes(const uint8_t * buffer, size_t length) override
  {
    return sendto(
      socket_fd_,
      buffer,
      length,
      0,
      reinterpret_cast<const sockaddr *>(&remote_addr_),
      sizeof(remote_addr_));
  }

  void log_read_error() override
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000,
      "UDP read failed: %s", std::strerror(errno));
  }

  void log_write_error() override
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000,
      "UDP write failed: %s", std::strerror(errno));
  }

private:
  std::string bind_host_;
  int bind_port_{14580};
  std::string remote_host_;
  int remote_port_{14540};
  int socket_fd_{-1};
  sockaddr_in remote_addr_{};
};

class MavlinkInjectorNode : public rclcpp::Node
{
public:
  MavlinkInjectorNode()
  : Node("mavlink_injector_node")
  {
    declare_parameter<std::string>("transport", "udp");
    declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<std::string>("udp_bind_host", "0.0.0.0");
    declare_parameter<int>("udp_bind_port", 14540);
    declare_parameter<std::string>("udp_remote_host", "127.0.0.1");
    declare_parameter<int>("udp_remote_port", 14580);
    declare_parameter<int>("local_system_id", 42);
    declare_parameter<int>("local_component_id", kMavCompIdOnboardComputer);
    declare_parameter<int>("target_system_id", 1);
    declare_parameter<int>("target_component_id", 1);
    declare_parameter<double>("inject_hz", 50.0);
    declare_parameter<double>("telemetry_hz", 50.0);

    body_rate_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/control/body_rate_cmd", 10,
      std::bind(&MavlinkInjectorNode::body_rate_callback, this, std::placeholders::_1));

    thrust_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/control/thrust_cmd", 10,
      std::bind(&MavlinkInjectorNode::thrust_callback, this, std::placeholders::_1));

    vehicle_command_sub_ = create_subscription<std_msgs::msg::String>(
      "/control/vehicle_command", 10,
      std::bind(&MavlinkInjectorNode::vehicle_command_callback, this, std::placeholders::_1));

    attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/vehicle/attitude", 10);

    baro_alt_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/vehicle/baro_alt", 10);

    const auto transport = get_parameter("transport").as_string();
    const auto local_system_id = static_cast<uint8_t>(get_parameter("local_system_id").as_int());
    const auto local_component_id = static_cast<uint8_t>(get_parameter("local_component_id").as_int());
    const auto target_system_id = static_cast<uint8_t>(get_parameter("target_system_id").as_int());
    const auto target_component_id = static_cast<uint8_t>(get_parameter("target_component_id").as_int());

    if (transport == "serial") {
      auto bridge = std::make_unique<RawSerialMavlinkBridge>(
        get_logger(),
        get_parameter("serial_device").as_string(),
        get_parameter("baudrate").as_int(),
        local_system_id,
        local_component_id,
        target_system_id,
        target_component_id);
      bridge->set_clock(get_clock());
      bridge_ = std::move(bridge);
    } else if (transport == "udp") {
      auto bridge = std::make_unique<UdpMavlinkBridge>(
        get_logger(),
        get_parameter("udp_bind_host").as_string(),
        get_parameter("udp_bind_port").as_int(),
        get_parameter("udp_remote_host").as_string(),
        get_parameter("udp_remote_port").as_int(),
        local_system_id,
        local_component_id,
        target_system_id,
        target_component_id);
      bridge->set_clock(get_clock());
      bridge_ = std::move(bridge);
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "unsupported transport '%s'. Expected 'serial' or 'udp'.",
        transport.c_str());
      throw std::runtime_error("unsupported transport");
    }

    connected_ = bridge_->connect();

    const auto inject_period =
      std::chrono::duration<double>(1.0 / get_parameter("inject_hz").as_double());
    inject_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(inject_period),
      std::bind(&MavlinkInjectorNode::inject_timer_callback, this));

    const auto telemetry_period =
      std::chrono::duration<double>(1.0 / get_parameter("telemetry_hz").as_double());
    telemetry_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(telemetry_period),
      std::bind(&MavlinkInjectorNode::telemetry_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "mavlink_injector_node started with transport=%s",
      transport.c_str());
  }

private:
  void body_rate_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    last_body_rate_cmd_ = *msg;
    has_body_rate_cmd_ = true;
  }

  void thrust_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_thrust_cmd_ = *msg;
    has_thrust_cmd_ = true;
  }

  void vehicle_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & command = msg->data;

    if (command == "arm") {
      bridge_->arm();
      RCLCPP_INFO(get_logger(), "vehicle command received: arm");
      return;
    }

    if (command == "disarm") {
      bridge_->disarm();
      RCLCPP_INFO(get_logger(), "vehicle command received: disarm");
      return;
    }

    if (command == "takeoff_stop") {
      bridge_->stop_takeoff();
      return;
    }

    RCLCPP_WARN(get_logger(), "unknown vehicle command: %s", command.c_str());
  }

  void inject_timer_callback()
  {
    if (!connected_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "FC transport is not connected. Injection is skipped.");
      return;
    }

    bridge_->send_body_rate_and_thrust(last_body_rate_cmd_, last_thrust_cmd_.data);
  }

  void telemetry_timer_callback()
  {
    if (!connected_) {
      return;
    }

    bridge_->poll();

    geometry_msgs::msg::Vector3Stamped attitude_msg;
    if (bridge_->take_attitude(attitude_msg)) {
      attitude_pub_->publish(attitude_msg);
    }

    std_msgs::msg::Float32 baro_alt_msg;
    if (bridge_->take_baro_alt(baro_alt_msg)) {
      baro_alt_pub_->publish(baro_alt_msg);
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_rate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thrust_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr baro_alt_pub_;
  rclcpp::TimerBase::SharedPtr inject_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;

  std::unique_ptr<FcBridge> bridge_;
  bool connected_{false};

  geometry_msgs::msg::Vector3Stamped last_body_rate_cmd_{};
  std_msgs::msg::Float32 last_thrust_cmd_{};
  bool has_body_rate_cmd_{true};
  bool has_thrust_cmd_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavlinkInjectorNode>());
  rclcpp::shutdown();
  return 0;
}
