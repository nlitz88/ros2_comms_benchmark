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
#include "timing_diagnostics/event_timing_diagnostic_task.hpp"
#include "comms_benchmark_interfaces/msg/message_timing_diagnostic.hpp"
#include <rclcpp/qos_overriding_options.hpp>

namespace comms_benchmark
{

BenchmarkSubscriber::BenchmarkSubscriber(const rclcpp::NodeOptions & options) 
: Node("benchmark_subscriber", options),
  num_images_received_(0),
  last_image_msg_timestamp_s(0),
  first_image(true)
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

    this->timing_diagnostic_task_ = std::make_shared<timing_diagnostics::EventTimingDiagnosticTask>(
        this->get_clock(),
        "BenchmarkSubscriberTimingDiagnosticTask"
    );


    /**
     * @brief Setup publisher options. Specifically, make the key QOS policies
     * configurable.
     * 
     */
    rclcpp::PublisherOptions publisher_options;
    publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions({
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Reliability,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::Reliability
    });

    this->message_timing_diagnostics_publisher_ = this->create_publisher<comms_benchmark_interfaces::msg::MessageTimingDiagnostic>(
        "~/message_timing",
        rclcpp::QoS(10).reliable(), // Default QoS
        publisher_options
    );

}

BenchmarkSubscriber::~BenchmarkSubscriber()
{
    // Destructor implementation, if needed.

}

void BenchmarkSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
{

    double current_time_s = this->get_clock()->now().seconds();
    // If this is the first image, just record the last timestamp so we can
    // compute elapsed time next image.
    if (first_image == true) {
        first_image = false;
        last_image_msg_timestamp_s = current_time_s;
    }
    else {
        // Publish the current time and elapsed time.
        comms_benchmark_interfaces::msg::MessageTimingDiagnostic message_timing_diagnostic;
        message_timing_diagnostic.callback_elapsed_time_s = current_time_s - last_image_msg_timestamp_s;
        message_timing_diagnostic.message_latency_s = current_time_s - rclcpp::Time(image_msg->header.stamp).seconds();
        this->message_timing_diagnostics_publisher_->publish(message_timing_diagnostic);
    }
    last_image_msg_timestamp_s = current_time_s;

    this->num_images_received_++;
    RCLCPP_INFO(this->get_logger(), "Received image %d", this->num_images_received_);

}

} // namespace comms_benchmark

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(comms_benchmark::BenchmarkSubscriber)