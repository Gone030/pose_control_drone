#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
      : Node("controller_node")
  {
    declare_parameter<double>("publish_hz", 20.0);
    declare_parameter<double>("hover_thrust", 0.72);
    declare_parameter<double>("kp_z", 0.05);
    declare_parameter<double>("ki_z", 0.00);
    declare_parameter<double>("kd_z", 0.015);
    declare_parameter<double>("thrust_min", 0.70);
    declare_parameter<double>("thrust_max", 0.75);
    declare_parameter<double>("vz_filter_alpha", 0.80);
    declare_parameter<double>("z_integral_limit", 0.3);

    command_pub_ = create_publisher<std_msgs::msg::String>("/control/vehicle_command", 10);
    body_rate_input_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/control/body_rate_input", 10,
        std::bind(&ControllerNode::body_rate_input_callback, this, std::placeholders::_1));

    hover_altitude_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/control/hover", 10,
        std::bind(&ControllerNode::hover_altitude_callback, this, std::placeholders::_1));

    baro_altitude_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/vehicle/baro_alt", 10,
        std::bind(&ControllerNode::baro_altitude_callback, this, std::placeholders::_1));

    body_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/control/body_rate_cmd", 10);

    thrust_pub_ = create_publisher<std_msgs::msg::Float32>("/control/thrust_cmd", 10);

    const auto period =
        std::chrono::duration<double>(1.0 / get_parameter("publish_hz").as_double());

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ControllerNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "controller_node started");
  }

private:
  void body_rate_input_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {

    last_body_rate_input_ = *msg;
  }

  void baro_altitude_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const double vz_filter_alpha = get_parameter("vz_filter_alpha").as_double();
    const double baro_altitude = static_cast<double>(msg->data);
    const rclcpp::Time stamp = now();

    if (!baro_reference_initialized_)
    {
      baro_reference_altitude_ = baro_altitude;
      current_z_ = 0.0;
      current_vz_ = 0.0;
      last_baro_altitude_ = baro_altitude;
      last_baro_stamp_ = stamp;
      baro_reference_initialized_ = true;
      has_altitude_measurement_ = true;

      RCLCPP_INFO(
          get_logger(),
          "baro reference initialized: abs=%.3f rel=0.000",
          baro_reference_altitude_);

      return;
    }

    const double relative_altitude = baro_altitude - baro_reference_altitude_;
    double vertical_velocity = current_vz_;

    if (std::isfinite(last_baro_altitude_))
    {
      const double dt = (stamp - last_baro_stamp_).seconds();

      if (dt > 1e-3)
      {
        const double raw_vertical_velocity = (baro_altitude - last_baro_altitude_) / dt;

        vertical_velocity =
            vz_filter_alpha * current_vz_ +
            (1.0 - vz_filter_alpha) * raw_vertical_velocity;
      }
    }

    current_z_ = relative_altitude;
    current_vz_ = vertical_velocity;
    last_baro_altitude_ = baro_altitude;
    last_baro_stamp_ = stamp;
    has_altitude_measurement_ = true;
  }

  void hover_altitude_callback(const std_msgs::msg::Float32::SharedPtr msg)

  {

    z_target_ = msg->data;

    std_msgs::msg::String st;

    st.data = "takeoff_stop";

    command_pub_->publish(st);
    hover_active_ = true;
    z_error_integral_ = 0.0;

    RCLCPP_INFO(
        get_logger(),
        "hover target updated: relative_z=%.3f (current_z=%.3f, current_vz=%.3f)",
        z_target_,
        current_z_,
        current_vz_);
  }

  void timer_callback()
  {

    const double publish_hz = get_parameter("publish_hz").as_double();
    const double hover_thrust = get_parameter("hover_thrust").as_double();
    const double kp_z = get_parameter("kp_z").as_double();
    const double ki_z = get_parameter("ki_z").as_double();
    const double kd_z = get_parameter("kd_z").as_double();
    const double thrust_min = get_parameter("thrust_min").as_double();
    const double thrust_max = get_parameter("thrust_max").as_double();
    const double z_integral_limit = get_parameter("z_integral_limit").as_double();

    geometry_msgs::msg::Vector3Stamped body_rate_msg = last_body_rate_input_;
    body_rate_msg.header.stamp = now();
    std_msgs::msg::Float32 thrust_msg;

    thrust_msg.data = static_cast<float>(hover_thrust);
    body_rate_pub_->publish(body_rate_msg);

    if (!has_altitude_measurement_ || !hover_active_)
    {
      thrust_pub_->publish(thrust_msg);
      return;
    }

    const double dt = 1.0 / publish_hz;
    const double z_error = z_target_ - current_z_;

    if (hover_active_ && z_target_ < -0.5 && z_error < 0.9)
    {
      RCLCPP_INFO(get_logger(), "landing command receive.");
      std_msgs::msg::String st;
      st.data = "disarm";
      command_pub_->publish(st);
    }

    z_error_integral_ += z_error * dt;
    z_error_integral_ = clamp(z_error_integral_, -z_integral_limit, z_integral_limit);

    const double thrust_correction =
        kp_z * z_error - kd_z * current_vz_ + ki_z * z_error_integral_;

    const double thrust = clamp(hover_thrust + thrust_correction, thrust_min, thrust_max);

    thrust_msg.data = static_cast<float>(thrust);
    thrust_pub_->publish(thrust_msg);
  }

  double clamp(double value, double limit) const
  {
    return std::clamp(value, -limit, limit);
  }

  double clamp(double value, double min_value, double max_value) const
  {
    return std::clamp(value, min_value, max_value);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_rate_input_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hover_altitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr baro_altitude_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Vector3Stamped last_body_rate_input_{};

  bool hover_active_{false};
  bool has_altitude_measurement_{false};
  bool baro_reference_initialized_{false};
  double z_target_{0.0};
  double current_z_{0.0};
  double current_vz_{0.0};
  double baro_reference_altitude_{0.0};
  double last_baro_altitude_{std::numeric_limits<double>::quiet_NaN()};

  rclcpp::Time last_baro_stamp_{0, 0, RCL_ROS_TIME};
  double z_error_integral_{0.0};
};

int main(int argc, char **argv)

{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ControllerNode>());

  rclcpp::shutdown();

  return 0;
}
