/**
 * @file benchmark_subscriber.hpp
 * @author Nathan Litzinger (nlitz88@gmail.com)
 * @brief Mock message subscriber component for benchmarking ROS2 communication
 * performance.
 * @version 0.1
 * @date 2025-06-26
 * 
 */

#include "comms_benchmark/benchmark_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace comms_benchmark
{

BenchmarkSubscriber::BenchmarkSubscriber(const rclcpp::NodeOptions & options) 
: Node("benchmark_subscriber", options),
  num_images_received_(0)
{

    // TODO: Declare and grab parameters.

    // TODO: Create a new subscriber instance.
    this->image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image",
        10,
        std::bind(&BenchmarkSubscriber::image_callback, this, std::placeholders::_1)
    );

    // To understand what the placeholder is doing with bind, see this tutorial:
    // https://www.geeksforgeeks.org/bind-function-placeholders-c/

}

BenchmarkSubscriber::~BenchmarkSubscriber()
{
    // Destructor implementation, if needed.

}

void BenchmarkSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{

    this->num_images_received_++;
    RCLCPP_INFO(this->get_logger(), "Received image %d", this->num_images_received_);

}

} // namespace comms_benchmark

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(comms_benchmark::BenchmarkSubscriber)