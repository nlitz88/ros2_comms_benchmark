/**
 * @file benchmark_publisher.cpp
 * @author Nathan Litzinger (nlitz88@gmail.com)
 * @brief ROS2 component for publishing a fixed number of image messages to
 * benchmark communication performance.
 * @version 0.1
 * @date 2025-06-15
 * 
 */

#include "comms_benchmark/benchmark_publisher.hpp"

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

namespace comms_benchmark
{

BenchmarkPublisher::BenchmarkPublisher(const rclcpp::NodeOptions & options)
: Node("benchmark_publisher", options),
  num_images_(0)
{
    
    // TODO: Declare and grab parameter values.
    // Number of images to publish. OR, ask for a publishing rate and duration.
    // I.e., 100 Hz for 10 seconds would be 1000 images.
    // Image resolution
    // TODO: We grab the value of these parameters right here, as this node is
    // not being written to support these parameters changing on the fly right
    // now
    num_images_ = this->declare_parameter<int>("num_images", 100);
    int publish_rate_hz = this->declare_parameter<int>("publish_rate_hz", 10);
    int image_width_px = this->declare_parameter<int>("image_width_px", 640);
    int image_height_px = this->declare_parameter<int>("image_height_px", 480);

    // QUESTION: Do we have to expose QOS policies via parameters? Or are these
    // already exposed? I feel like the profile is already exposed, but not sure
    // about the individual policies. Have to run `ros2 param list` to check
    // once I bring up the node once.

    // Create a publisher instance.
    // TODO: We should explicitly define a QOS profile here, built from scratch
    // using individual, parameterized policies--as that is one of the points of
    // this node. For now, a simple publisher is fine.
    this->image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "image",
        10
    );

    // Create a timer instance.
    // NOTE: I'm pretty sure we explicitly want to use create_timer here instead
    // of create_wall_timer, as create_wall_timer strictly uses the system wall
    // clock. create_timer uses whatever clock the node is using, which means it
    // can switch between the system clock and a ROS /clock topic. We care about
    // this, as if we're using sim time, we want our timer to operate on the
    // same time scale as the rest of the system.
    // Compute the timer period based on the provided publish rate.
    // https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/create_timer.hpp#L36-L52
    int publish_period_ms = static_cast<int>((1.0 / publish_rate_hz) * 1000);
    this->image_timer_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_clock(),
        std::chrono::milliseconds(publish_period_ms),
        std::bind(&BenchmarkPublisher::image_timer_callback, this)
    );
    
    // Allocate the image to be published.
    cv::Mat image = cv::Mat(
        image_height_px,
        image_width_px,
        CV_8UC3, // 8-bit unsigned integer, 3 channels (RGB)
        cv::Scalar(0, 255, 0) // Initialize to black
    );
    
    // Create a cv_bridge CvPtr to convert the OpenCV image to a ROS message.
    cv_bridge::CvImage cv_image;
    cv_image.header.frame_id = "camera_frame";
    cv_image.header.stamp = this->get_clock()->now();
    cv_image.encoding = sensor_msgs::image_encodings::RGB8;
    cv_image.image = image;

    // Convert the OpenCV image to a ROS message.
    this->image_msg_ = cv_image.toImageMsg();

}

BenchmarkPublisher::~BenchmarkPublisher()
{

}

void BenchmarkPublisher::image_timer_callback()
{

    // TODO: Update this function so that it writes a number to each debug
    // image and publishes that.

    // Update the header timestamp for the image message before publishing.
    this->image_msg_->header.stamp = this->get_clock()->now();
    this->image_publisher_->publish(*(this->image_msg_));
    // RCLCPP_INFO(this->get_logger(), "Published image with timets")
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Published image " << this->num_images_ << " with timestamp "
        << this->image_msg_->header.stamp.sec << "."
        << this->image_msg_->header.stamp.nanosec
    );
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Example warning message for testing purposes."
    );
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Example error message for testing purposes."
    );
    std::cout << "Test print to standard out!" << std::endl;
}

} // namespace comms_benchmark

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// Taken from
// https://github.com/ros2/demos/blob/7dffb2cc9b6e6711c8877572cb8bebbb7dae74b1/composition/src/talker_component.cpp#L59-L62
RCLCPP_COMPONENTS_REGISTER_NODE(comms_benchmark::BenchmarkPublisher)