/**
 * @file benchmark_subscriber.hpp
 * @author Nathan Litzinger (nlitz88@gmail.com)
 * @brief Mock message subscriber component for benchmarking ROS2 communication
 * performance.
 * @version 0.1
 * @date 2025-06-26
 * 
 */

#ifndef COMMS_BENCHMARK_BENCHMARK_SUBSCRIBER_HPP_
#define COMMS_BENCHMARK_BENCHMARK_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>

namespace comms_benchmark
{

class BenchmarkSubscriber : public rclcpp::Node
{

public:

    /**
     * @brief Constructor for the comms_benchmark::BenchmarkSubscriber.
     * @param options Additional options to control creation of the node.
     */
    explicit BenchmarkSubscriber(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructor for the comms_benchmark::BenchmarkSubscriber.
     */
    ~BenchmarkSubscriber();

protected:

    /**
     * @brief Pointer for the image subscriber. This will be allocated in the
     * constructor.
     */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

    /**
     * @brief Callback function for the image subscriber. This will be invoked
     * whenever a new image message is received.
     * @param image_msg The received image message.
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg);

    /**
     * @brief Total number of images received.
     */
    int num_images_received_;

};

}

#endif // COMMS_BENCHMARK_BENCHMARK_SUBSCRIBER_HPP_