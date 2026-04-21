#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <cstdio>
#include <cstring>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

class TeleopBodyrateNode : public rclcpp::Node
{
public:
  TeleopBodyrateNode()
  : Node("teleop_bodyrate_node")
  {
    declare_parameter<double>("pitch_rate_cmd", 0.25);
    declare_parameter<double>("yaw_rate_cmd", 0.5);
    declare_parameter<double>("publish_hz", 20.0);

    pitch_rate_cmd_ = get_parameter("pitch_rate_cmd").as_double();
    yaw_rate_cmd_ = get_parameter("yaw_rate_cmd").as_double();

    body_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/control/teleop_body_rate_raw", 10);
    hover_alt_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/control/hover", 10);

    atteck_cmd_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/control/atteck", 10);

    vehicle_command_pub_ = create_publisher<std_msgs::msg::String>(
      "/control/vehicle_command", 10);

    const auto period =
      std::chrono::duration<double>(1.0 / get_parameter("publish_hz").as_double());

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TeleopBodyrateNode::timer_callback, this));

    setup_terminal();

    RCLCPP_INFO(get_logger(), "teleop_bodyrate_node started");
    RCLCPP_INFO(get_logger(), "w/s: pitch  a/d: yaw  q/e: roll  u/i: alt  y: arm  m: quit");
  }

  ~TeleopBodyrateNode() override
  {
    restore_terminal();
  }

private:
  void setup_terminal()
  {
    tcgetattr(STDIN_FILENO, &original_termios_);
    termios raw = original_termios_;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    terminal_initialized_ = true;
  }

  void restore_terminal()
  {
    if (terminal_initialized_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      terminal_initialized_ = false;
    }
  }

  bool read_key(char & c)
  {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &set)) {
      return ::read(STDIN_FILENO, &c, 1) == 1;
    }
    return false;
  }

  void timer_callback()
  {
    double roll_rate = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;
    std_msgs::msg::Float32 hv;

    char key = 0;
    while (read_key(key)) {
      last_key_ = key;
    }

    switch (last_key_) {
      case 'w':
      case 'W':
        pitch_rate = -pitch_rate_cmd_;
        break;
      case 's':
      case 'S':
        pitch_rate = pitch_rate_cmd_;
        break;
      case 'a':
      case 'A':
        yaw_rate = -yaw_rate_cmd_;
        break;
      case 'd':
      case 'D':
        yaw_rate = yaw_rate_cmd_;
        break;
      case 'q':
      case 'Q':
        roll_rate = -roll_rate_cmd_;
        break;
      case 'e':
      case 'E':
        roll_rate = roll_rate_cmd_;
        break;
      case 'u':
      case 'U':
        last_hover_alt_ += hover_alt_cmd_;
        hv.data = last_hover_alt_;
        RCLCPP_INFO(get_logger(), "alt : %f", last_hover_alt_);
        break;
      case 'i':
      case 'I':
        last_hover_alt_ -= hover_alt_cmd_;
        hv.data = last_hover_alt_;
        RCLCPP_INFO(get_logger(), "alt : %f", last_hover_alt_);
        break;
      case 'o':
      case 'O':
        atteck_cmd_ = true;
        break;
      case 'p':
      case 'P':
        atteck_cmd_ = false;
        break;
      case 'm':
      case 'M':
        RCLCPP_INFO(get_logger(), "quit requested");
        rclcpp::shutdown();
        return;
      case 'y':
      case 'Y':
      {
        std_msgs::msg::String cmd;
        cmd.data = "arm";
        vehicle_command_pub_->publish(cmd);
        RCLCPP_INFO(get_logger(), "arm command published");
        break;
      }
      default:
        break;
    }

    last_key_ = 0;

    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = now();
    msg.vector.x = roll_rate;
    msg.vector.y = pitch_rate;
    msg.vector.z = yaw_rate;
    body_rate_pub_->publish(msg);

    if(last_hover_alt_ != last_hover_cmd_){
      hover_alt_pub_->publish(hv);
      last_hover_cmd_ = last_hover_alt_;
    }
    std_msgs::msg::Bool at;
    at.data = atteck_cmd_;
    atteck_cmd_pub_->publish(at);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hover_alt_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr atteck_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  termios original_termios_{};
  bool terminal_initialized_{false};

  double pitch_rate_cmd_{0.25};
  double yaw_rate_cmd_{0.5};
  double roll_rate_cmd_{0.25};
  double hover_alt_cmd_{1.0};
  double last_hover_alt_{0.0};
  double last_hover_cmd_{0.0};
  bool atteck_cmd_{false};
  char last_key_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopBodyrateNode>());
  rclcpp::shutdown();
  return 0;
}
