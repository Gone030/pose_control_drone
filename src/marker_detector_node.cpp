#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class MarkerDetectorNode : public rclcpp::Node
{
public:
  MarkerDetectorNode()
  : Node("marker_detector_node"), frame_count_(0), detected_count_(0)
  {
    declare_parameter<double>("ref_x_ratio", 0.5);
    declare_parameter<double>("ref_y_ratio", 0.30);
    declare_parameter<double>("detect_scale", 1.0);

    ref_x_ratio_ = get_parameter("ref_x_ratio").as_double();
    ref_y_ratio_ = get_parameter("ref_y_ratio").as_double();
    detect_scale_ = get_parameter("detect_scale").as_double();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&MarkerDetectorNode::imageCallback, this, std::placeholders::_1));

    atteck_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/control/atteck", 10,
      std::bind(&MarkerDetectorNode::atteckCallback, this, std::placeholders::_1));


    detected_pub_ = create_publisher<std_msgs::msg::Bool>("/marker/detected", 10);
    error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/marker/error", 10);
    marker_size_pub_ = create_publisher<std_msgs::msg::Float32>("/marker/size", 10);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MarkerDetectorNode::timerCallback, this));

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(get_logger(), "marker_detector_node started. subscribing: /camera/image_raw");
  }

private:
  void atteckCallback(const std_msgs::msg::Bool::SharedPtr msg){
    atteck_cmd_ = msg->data;
  }
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if(!atteck_cmd_) return;
    frame_count_++;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cv_bridge exception: %s", e.what());
      return;
    }

    const int img_w = cv_ptr->image.cols;
    const int img_h = cv_ptr->image.rows;

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat gray_small;
    cv::resize(
      gray,
      gray_small,
      cv::Size(),
      detect_scale_,
      detect_scale_,
      cv::INTER_LINEAR);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners_small;
    std::vector<std::vector<cv::Point2f>> rejected_small;

    cv::aruco::detectMarkers(
      gray_small,
      dictionary_,
      corners_small,
      ids,
      detector_params_,
      rejected_small);

    std_msgs::msg::Bool detected_msg;
    geometry_msgs::msg::Vector3Stamped error_msg;
    std_msgs::msg::Float32 size_msg;

    error_msg.header.stamp = msg->header.stamp;

    if (!ids.empty()) {
      detected_count_++;
      detected_msg.data = true;

      const auto & c_small = corners_small[0];
      std::vector<cv::Point2f> c;
      c.reserve(c_small.size());
      for (const auto & p : c_small) {
        c.emplace_back(
          static_cast<float>(p.x / detect_scale_),
          static_cast<float>(p.y / detect_scale_));
      }

      double marker_cx = 0.0;
      double marker_cy = 0.0;
      for (const auto & p : c) {
        marker_cx += p.x;
        marker_cy += p.y;
      }
      marker_cx /= 4.0;
      marker_cy /= 4.0;

      const double ref_x = ref_x_ratio_ * static_cast<double>(img_w);
      const double ref_y = ref_y_ratio_ * static_cast<double>(img_h);

      const double ex = ref_x - marker_cx;
      const double ey = ref_y - marker_cy;

      error_msg.vector.x = ex;
      error_msg.vector.y = ey;
      error_msg.vector.z = 0.0;

      const double marker_area_px = std::fabs(cv::contourArea(c));
      size_msg.data = static_cast<float>(std::sqrt(marker_area_px));
    } else {
      detected_msg.data = false;
      error_msg.vector.x = 0.0;
      error_msg.vector.y = 0.0;
      error_msg.vector.z = 0.0;
      size_msg.data = 0.0f;
    }

    detected_pub_->publish(detected_msg);
    error_pub_->publish(error_msg);
    marker_size_pub_->publish(size_msg);
  }

  void timerCallback()
  {
    RCLCPP_INFO(
      get_logger(),
      "frames=%zu detected_frames=%zu",
      frame_count_,
      detected_count_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr atteck_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr marker_size_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  size_t frame_count_;
  size_t detected_count_;

  bool atteck_cmd_{false};
  double ref_x_ratio_{0.5};
  double ref_y_ratio_{0.30};
  double detect_scale_{1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
