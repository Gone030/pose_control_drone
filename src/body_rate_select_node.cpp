#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class BodyRateSelectNode : public rclcpp::Node
{
public:
  BodyRateSelectNode()
  : Node("body_rate_select_node")
  {
    vision_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/control/vision_body_rate_raw", 10,
      std::bind(&BodyRateSelectNode::vision_callback, this, std::placeholders::_1));

    teleop_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/control/teleop_body_rate_raw", 10,
      std::bind(&BodyRateSelectNode::teleop_callback, this, std::placeholders::_1));

    marker_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/marker/detected", 10,
      std::bind(&BodyRateSelectNode::marker_detected_callback, this, std::placeholders::_1));

    body_rate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/control/body_rate_input", 10);

    RCLCPP_INFO(get_logger(), "body_rate_select_node started");
  }

private:
  void vision_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    last_vision_msg_ = msg;
    if (marker_detected_) {
      body_rate_pub_->publish(*msg);
    }
  }

  void teleop_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    last_teleop_msg_ = msg;
    if (!marker_detected_) {
      body_rate_pub_->publish(*msg);
    }
  }

  void marker_detected_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool previous_detected = marker_detected_;
    marker_detected_ = msg->data;

    if (previous_detected != marker_detected_) {
      RCLCPP_INFO(
        get_logger(),
        "input source changed: %s",
        marker_detected_ ? "vision_body_rate_raw" : "teleop_body_rate_raw");
    }

    publish_selected_latest();
  }

  void publish_selected_latest()
  {
    if (marker_detected_) {
      if (last_vision_msg_) {
        body_rate_pub_->publish(*last_vision_msg_);
      }
      return;
    }

    if (last_teleop_msg_) {
      body_rate_pub_->publish(*last_teleop_msg_);
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr vision_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr marker_detected_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_rate_pub_;

  geometry_msgs::msg::Vector3Stamped::SharedPtr last_vision_msg_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr last_teleop_msg_;
  bool marker_detected_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyRateSelectNode>());
  rclcpp::shutdown();
  return 0;
}
