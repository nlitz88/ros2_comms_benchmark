
/**
 * @file event_timing_diagnostic_task.hpp
 * @author Nathan Litzinger (nlitz88@gmail.com)
 * @brief Diagnostic task for recording detailed timing diagnostic for events
 * that occur in a node.
 * @version 0.1
 * @date 2025-07-08
 * 
 */

#include "timing_diagnostics/event_timing_diagnostic_task.hpp"

#include "rclcpp/time.hpp"

namespace timing_diagnostics
{

EventTimingDiagnosticTask::EventTimingDiagnosticTask(
    const rclcpp::Clock::SharedPtr clock,
    const std::string & name)
: DiagnosticTask(name), clock_(clock)
{
    // 

}

void EventTimingDiagnosticTask::run(
    diagnostic_updater::DiagnosticStatusWrapper & stat)
{

}

void EventTimingDiagnosticTask::log_event(const std::string & event_name)
{

    // TODO: This function should grab the current time using this instance's
    // clock, and populate a DiagnosticStatus message with it. It should just
    // create a field called "current_time" and set the value to that timestamp
    // double.
    
    rclcpp::Time current_time = this->clock_->now();
    // Construct a DiagnosticStatus message.


    // Further, it could create another field called "elapsed_time" where it
    // computes the difference between the current time and the last time this
    // particular event occured.
    
    // This diagnostic status message should then be placed in a vector of
    // diagnostic status messages corresponding to this particular event_name.

    // Use a map to organize these, like a python dictionary. I.e., a map of
    // vectors of diagnosticStatus messages.

    // In the run function, it will basically empty each map entry's vector into
    // one big diagnosticStatusArray message and publish that.




}

// EventTimingDiagnosticTask::~EventTimingDiagnosticTask()
// {
//     // Destructor implementation, if needed
// }

} // namespace timing_diagnostics