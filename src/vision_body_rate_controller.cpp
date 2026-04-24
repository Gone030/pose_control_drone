#include <algorithm>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

class VisionBodyRateControllerNode : public rclcpp::Node
{
public:
  VisionBodyRateControllerNode()
  : Node("vision_body_rate_controller_node")
  {
    declare_parameter<double>("publish_hz", 30.0);
    declare_parameter<double>("lost_timeout_sec", 0.3);

    // 비선형 제어
    declare_parameter<double>("roll_min_rate", 0.12);
    declare_parameter<double>("roll_curve_gamma", 1.65);

    // 브레이크
    declare_parameter<double>("base_brake_enter_px", 40.0);
    declare_parameter<double>("brake_enter_roll_gain", 180.0);
    declare_parameter<double>("brake_cmd_mag", 0.25);
    declare_parameter<int>("brake_hold_ticks", 6);
    declare_parameter<double>("brake_release_roll_rad", 0.07);
    declare_parameter<double>("brake_size_threshold_px", 120.0);

    declare_parameter<int>("reacquire_holdoff_ticks", 12);

    // 트래킹
    declare_parameter<double>("roll_kp", 0.004);
    declare_parameter<double>("pitch_kp", 0.004);
    declare_parameter<double>("max_roll_rate", 0.17);
    declare_parameter<double>("max_pitch_rate", 0.15);
    declare_parameter<double>("deadband_x_px", 5.0);
    declare_parameter<double>("deadband_y_px", 10.0);


    publish_hz_ = get_parameter("publish_hz").as_double();
    lost_timeout_sec_ = get_parameter("lost_timeout_sec").as_double();

    roll_kp_ = get_parameter("roll_kp").as_double();
    pitch_kp_ = get_parameter("pitch_kp").as_double();
    max_roll_rate_ = get_parameter("max_roll_rate").as_double();
    max_pitch_rate_ = get_parameter("max_pitch_rate").as_double();
    deadband_x_px_ = get_parameter("deadband_x_px").as_double();
    deadband_y_px_ = get_parameter("deadband_y_px").as_double();

    roll_min_rate_ = get_parameter("roll_min_rate").as_double();
    roll_curve_gamma_ = get_parameter("roll_curve_gamma").as_double();

    base_brake_enter_px_ = get_parameter("base_brake_enter_px").as_double();
    brake_enter_roll_gain_ = get_parameter("brake_enter_roll_gain").as_double();
    brake_cmd_mag_ = get_parameter("brake_cmd_mag").as_double();
    brake_hold_ticks_ = get_parameter("brake_hold_ticks").as_int();
    brake_release_roll_rad_ = get_parameter("brake_release_roll_rad").as_double();
    brake_size_threshold_px_ = get_parameter("brake_size_threshold_px").as_double();
    reacquire_holdoff_ticks_ = get_parameter("reacquire_holdoff_ticks").as_int();

    marker_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/marker/detected", 10,
      std::bind(&VisionBodyRateControllerNode::detectedCallback, this, std::placeholders::_1));

    marker_error_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/marker/error", 10,
      std::bind(&VisionBodyRateControllerNode::errorCallback, this, std::placeholders::_1));

    marker_size_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/marker/size", 10,
      std::bind(&VisionBodyRateControllerNode::sizeCallback, this, std::placeholders::_1));

    attitude_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/vehicle/attitude", 10,
      std::bind(&VisionBodyRateControllerNode::attitudeCallback, this, std::placeholders::_1));

    atteck_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/control/atteck", 10,
      std::bind(&VisionBodyRateControllerNode::atteckCallback, this, std::placeholders::_1));

    body_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/control/vision_body_rate_raw", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&VisionBodyRateControllerNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "vision_body_rate_controller_node started");
  }

private:
  enum class Mode
  {
    IDLE = 0,
    TRACK = 1,
    BRAKE = 2
  };

  void atteckCallback(const std_msgs::msg::Bool::SharedPtr msg){
    atteck_cmd_ = msg->data;
  }

  void detectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    marker_detected_ = msg->data;
    if (marker_detected_) {
      last_detected_stamp_ = now();
    }
  }

  void errorCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    ex_ = msg->vector.x;
    ey_ = msg->vector.y;
    last_error_stamp_ = now();
  }

  void sizeCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    marker_size_px_ = static_cast<double>(msg->data);
  }

  void attitudeCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    roll_rad_ = static_cast<double>(msg->vector.x);
    pitch_rad_ = static_cast<double>(msg->vector.y);
    yaw_rad_ = static_cast<double>(msg->vector.z);
    (void)roll_rad_;
    (void)pitch_rad_;
    (void)yaw_rad_;
  }

  bool hasRecentDetection() const
  {
    if (!marker_detected_) {
      return false;
    }
    return (now() - last_detected_stamp_).seconds() <= lost_timeout_sec_;
  }

  Mode determineMode() const
  {
    if (!hasRecentDetection() && !atteck_cmd_) {
      return Mode::IDLE;
    }
    if (marker_size_px_ >= brake_size_threshold_px_) {
      return Mode::BRAKE;
    }
    return Mode::TRACK;
  }

  double applyDeadband(double value, double threshold) const
  {
    return (std::fabs(value) < threshold) ? 0.0 : value;
  }

  double clamp(double value, double limit) const
  {
    return std::clamp(value, -limit, limit);
  }

  geometry_msgs::msg::Vector3 computeTrackCommand() const
  {
    geometry_msgs::msg::Vector3 cmd{};

    const int img_w = 320;  // 현재 카메라 기준 해상도에 맞춤
    const double roll_rate = computeNonlinearRollRate(ex_, img_w);

    const double ey_db = applyDeadband(ey_, deadband_y_px_);

    cmd.x = -roll_rate;
    cmd.y = clamp(pitch_kp_ * ey_db, max_pitch_rate_);
    cmd.z = 0.0;
    return cmd;
  }

  geometry_msgs::msg::Vector3 computeBrakeCommand() const
  {
    geometry_msgs::msg::Vector3 cmd{};
    cmd.x = brake_dir_ * brake_cmd_mag_;
    cmd.y = 0.0;
    cmd.z = 0.0;
    return cmd;
  }

  double computeNonlinearRollRate(double ex_px, int img_w) const
  {
    const double abs_ex = std::fabs(ex_px);
    if (abs_ex <= deadband_x_px_) {
      return 0.0;
    }

    const double effective_error = abs_ex - deadband_x_px_;
    const double max_effective_error =
        std::max(1.0, (static_cast<double>(img_w) * 0.5) - deadband_x_px_);
    const double normalized_error =
        std::clamp(effective_error / max_effective_error, 0.0, 1.0);

    const double curved = std::pow(normalized_error, roll_curve_gamma_);
    const double shaped_mag = max_roll_rate_ * curved;
    const double applied_mag = std::max(roll_min_rate_, shaped_mag);

    return std::copysign(applied_mag, ex_px);
  }

  bool shouldExitBrake() const
  {
    if (!hasRecentDetection()) {
      return true;
    }

    // 1. 마커 부호 뒤집힘
    if (brake_entry_ex_ * ex_ < 0.0) {
      return true;
    }

    // 1.5. roll 부호 뒤집힘
    if (brake_entry_roll_ * roll_rad_ < 0.0) {

      return true;

    }

    // 2. ex가 다시 커지기 시작함
    if (std::abs(ex_) > std::abs(brake_prev_ex_) + 5.0) {
      return true;
    }

    return false;
  }

  void timerCallback()
  {
    geometry_msgs::msg::Vector3Stamped out_msg;
    out_msg.header.stamp = now();
    if (!atteck_cmd_) return;

    const bool detected_now = hasRecentDetection();

    if (!detected_now) {
      mode_ = Mode::IDLE;
      brake_ticks_left_ = 0;
      brake_dir_ = 0.0;
      out_msg.vector.x = 0.0;
      out_msg.vector.y = 0.0;
      out_msg.vector.z = 0.0;
      body_rate_pub_->publish(out_msg);
      return;
    }
    if (!prev_detected_ && detected_now) {
      reacquire_ticks_left_ = reacquire_holdoff_ticks_;
    }

    prev_detected_ = detected_now;

    if (reacquire_ticks_left_ > 0) {
      reacquire_ticks_left_--;
    }

    if (mode_ == Mode::IDLE) {
      mode_ = Mode::TRACK;
    }

    if (mode_ == Mode::TRACK) {
      const auto track_cmd = computeTrackCommand();

      const double effective_brake_enter_px = base_brake_enter_px_ + brake_enter_roll_gain_ * std::abs(roll_rad_);
      const bool near_center = std::abs(ex_) < effective_brake_enter_px;
      const bool tilted = std::abs(roll_rad_) > brake_release_roll_rad_;
      const bool has_roll_cmd = std::abs(track_cmd.x) > 1e-6;

      if (reacquire_ticks_left_ == 0 && near_center && tilted && has_roll_cmd) {
        mode_ = Mode::BRAKE;
        brake_ticks_left_ = brake_hold_ticks_;
        brake_dir_ = -std::copysign(1.0, roll_rad_);

        brake_entry_ex_ = ex_;
        brake_prev_ex_ = ex_;
        brake_entry_roll_ = roll_rad_;
        brake_started_ = true;

        out_msg.vector = computeBrakeCommand();
      } else {
        out_msg.vector = track_cmd;
      }
    }
    else if (mode_ == Mode::BRAKE) {
      out_msg.vector = computeBrakeCommand();

      if (brake_ticks_left_ > 0) {
        brake_ticks_left_--;
      }

      if (shouldExitBrake()) {
        mode_ = Mode::TRACK;
        brake_dir_ = 0.0;
        brake_started_ = false;
        reacquire_ticks_left_ = reacquire_holdoff_ticks_;
        out_msg.vector = computeTrackCommand();
      } else {
        brake_prev_ex_ = ex_;
      }

      if (brake_ticks_left_ <= 0 && std::abs(roll_rad_) < brake_release_roll_rad_) {
        mode_ = Mode::TRACK;
        brake_dir_ = 0.0;
        brake_started_ = false;
        reacquire_ticks_left_ = reacquire_holdoff_ticks_;
        out_msg.vector = computeTrackCommand();
      }
    }

    body_rate_pub_->publish(out_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "mode=%d ex=%.1f roll_rad=%.3f reacq_left=%d brake_dir=%.1f brake_left=%d cmd_x=%.3f",
      static_cast<int>(mode_),
      ex_,
      roll_rad_,
      reacquire_ticks_left_,
      brake_dir_,
      brake_ticks_left_,
      out_msg.vector.x);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr marker_detected_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr marker_error_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr marker_size_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr atteck_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_rate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool marker_detected_{false};
  bool atteck_cmd_{false};

  double ex_{0.0};
  double ey_{0.0};
  double marker_size_px_{0.0};
  double roll_rad_{0.0};
  double pitch_rad_{0.0};
  double yaw_rad_{0.0};
  rclcpp::Time last_detected_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_error_stamp_{0, 0, RCL_ROS_TIME};


  double publish_hz_{30.0};
  double lost_timeout_sec_{0.3};
  double roll_kp_{0.004};
  double pitch_kp_{0.004};
  double max_roll_rate_{0.35};
  double max_pitch_rate_{0.35};
  double deadband_x_px_{10.0};
  double deadband_y_px_{10.0};
  double brake_size_threshold_px_{120.0};

  Mode mode_{Mode::IDLE};

  // 비선형
  double roll_min_rate_{0.08};
  double roll_curve_gamma_{2.0};

  // 브레이크
  double brake_enter_px_{90.0};
  double base_brake_enter_px_{0.0};
  double brake_enter_roll_gain_{0.0};
  double brake_cmd_mag_{0.20};
  int brake_hold_ticks_{6};
  double brake_entry_roll_{0.0};
  double brake_release_roll_rad_{0.08};

  double brake_dir_{0.0};
  int brake_ticks_left_{0};

  double brake_entry_ex_{0.0};
  double brake_prev_ex_{0.0};
  bool brake_started_{false};

  // 브레이크 재진입
  int reacquire_holdoff_ticks_{5};
  int reacquire_ticks_left_{0};

  bool prev_detected_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionBodyRateControllerNode>());
  rclcpp::shutdown();
  return 0;
}


