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
#include "timing_diagnostics/event_timing_diagnostic_task.hpp"
#include "comms_benchmark_interfaces/msg/message_timing_diagnostic.hpp"

#include <map>
#include <vector>

namespace comms_benchmark
{

/**
 * @brief Struct for maintaining the details of a timing event. This includes
 * the timestamp that the event diagnostics was created (header_timestamp), the
 * event's actual timestamp (which will be the same as the event timestamp), and
 * the elapsed time between this event and the last).
 * 
 */
typedef struct event {
    double header_timestamp_s;
    double event_timestamp_s;
    double event_elapsed_time_s;
} event_t;

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

    timing_diagnostics::EventTimingDiagnosticTask::SharedPtr timing_diagnostic_task_;


    /**
     * @brief Map from each event to a vector of event records.
     * 
     */
    std::map<std::string, std::vector<event_t>> event_buffer;

    /**
     * @brief Create a timer that will periodically wake up and flush the
     * event_buffer map.
     * 
     */
    rclcpp::TimerBase::SharedPtr flush_events_timer;

    /**
     * @brief Create a publisher that the events will be published with.
     * 
     */
    rclcpp::Publisher<comms_benchmark_interfaces::msg::MessageTimingDiagnostic>::SharedPtr message_timing_diagnostics_publisher_;

    double last_image_msg_timestamp_s;
    bool first_image;

}; // class BenchmarkSubscriber

} // namespace comms_benchmark

#endif // COMMS_BENCHMARK_BENCHMARK_SUBSCRIBER_HPP_