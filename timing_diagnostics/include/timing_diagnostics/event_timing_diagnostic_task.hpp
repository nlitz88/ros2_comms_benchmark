/**
 * @file event_timing_diagnostic_task.hpp
 * @author Nathan Litzinger (nlitz88@gmail.com)
 * @brief Diagnostic task for recording detailed timing diagnostic for events
 * that occur in a node.
 * @version 0.1
 * @date 2025-07-08
 * 
 */

#ifndef EVENT_TIMING_DIAGNOSTIC_TASK_HPP_
#define EVENT_TIMING_DIAGNOSTIC_TASK_HPP_

#include <rclcpp/rclcpp.hpp>
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace timing_diagnostics
{

// Based on
// https://github.com/ros/diagnostics/blob/7021f83f92251d08c075a31fc5d8ada1928ee247/diagnostic_updater/include/diagnostic_updater/update_functions.hpp#L270


class EventTimingDiagnosticTask : public diagnostic_updater::DiagnosticTask
{

public:
    RCLCPP_SMART_PTR_DEFINITIONS(EventTimingDiagnosticTask)

    // TODO: Declare constructor and destructor.
    explicit EventTimingDiagnosticTask(
        const rclcpp::Clock::SharedPtr clock,
        const std::string & name);

    virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override;
    
    void log_event(const std::string & event_name);

// TODO: Declare methods for logging events.

// TODO: Do we need to redeclare the functions from the DiagnosticTask base
// class that we will override?

// ALSO? The diagnostic updater is a header only library. Should this also be a
// header only library? 

private:

    /**
     * @brief Pointer to the clock that will be used to record the times an
     * event takes place.
     */
    rclcpp::Clock::SharedPtr clock_;
    
    /**
     * @brief Map of event name strings to their corresponding event
     * diagnostics. Each event will have a vector of DiagnosticStatus messages,
     * where a new DiagnosticStatus message will be created/recorded for each
     * call to log_event() with that event's name.
     */
    std::map<std::string, std::vector<diagnostic_msgs::msg::DiagnosticStatus>> events_;

}; // class EventTimingDiagnosticTask

} // namespace timing_diagnostics

#endif // EVENT_TIMING_DIAGNOSTIC_TASK_HPP_