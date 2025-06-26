/**
 * @file benchmark_publisher.hpp
 * @author Nathan Litzinger (nlitz88@gmail.com)
 * @brief ROS2 component for publishing a fixed number of image messages to
 * benchmark communication performance.
 * @version 0.1
 * @date 2025-06-15
 * 
 */

#ifndef COMMS_BENCHMARK_BENCHMARK_PUBLISHER_HPP_
#define COMMS_BENCHMARK_BENCHMARK_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>

namespace comms_benchmark
{

class BenchmarkPublisher : public rclcpp::Node
{

public:

    /**
     * @brief Constructor for the comms_benchmark::BenchmarkPublisher.
     * @param options Additional options to control creation of the node.
     */
    explicit BenchmarkPublisher(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructor for the comms_benchmark::BenchmarkPublisher.
     */
    ~BenchmarkPublisher();

protected:

    /**
     * @brief Pointer for the image publisher. This will be allocated in the
     * constructor.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    /**
     * @brief Pointer for the timer that will be used to publish images at a
     * fixed rate specified by the user.
     */
    rclcpp::TimerBase::SharedPtr image_timer_;
    
    /**
     * @brief Timer callback function. This will be invoked by the timer and
     * publish a new image.
     */
    void image_timer_callback();

    /**
     * @brief Total number of images to publish.
     */
    int num_images_;
    
    /**
     * @brief Image message to be published. This will be created in the
     * constructor based on the provided image parameters.
     */
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    

}; // class BenchmarkPublisher

} // namespace comms_benchmark

#endif  // COMMS_BENCHMARK_BENCHMARK_PUBLISHER_HPP_
