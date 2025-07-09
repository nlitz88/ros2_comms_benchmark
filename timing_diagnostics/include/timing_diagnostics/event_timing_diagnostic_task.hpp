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



// TODO: Declare methods for logging events.

// TODO: Do we need to redeclare the functions from the DiagnosticTask base
// class that we will override?

// ALSO? The diagnostic updater is a header only library. Should this also be a
// header only library? 

private:

    // Define a map to map different event names to their respective event times.
    // The actual event times will be maintained as a vector
    // DiagnosticStatusWrappers or messages, or something like that.

    // Define a clock to use for timing events.
    rclcpp::Clock::SharedPtr clock_;

}; // class EventTimingDiagnosticTask

} // namespace timing_diagnostics

#endif // EVENT_TIMING_DIAGNOSTIC_TASK_HPP_