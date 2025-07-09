
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
    // Implementation of the run method to log events.
    // This method will be called periodically by the diagnostic updater.
    
    // Example: Log the current time as an event.
    // stat.add("Current Time", clock_->now().to_string());
    
    // Additional event logging can be added here.
}

// EventTimingDiagnosticTask::~EventTimingDiagnosticTask()
// {
//     // Destructor implementation, if needed
// }

} // namespace timing_diagnostics