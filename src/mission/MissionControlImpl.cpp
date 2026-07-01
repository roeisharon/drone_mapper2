#include <drone_mapper/mission/MissionControlImpl.h>

#include <exception>
#include <utility>
#include <vector>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

// Drives the step loop for one mission:
//   for i in [0, max_steps):
//       result = drone_control_.step()
//       Completed → break;  Error → record + continue;  Continue → next step.
// Final status priority: Completed > Error > MaxSteps. The output map is always saved afterwards
// (even on failure) so the caller has a map. The loop is wrapped so an unexpected throw becomes an
// error entry rather than crashing (EH-1, EH-5); errors do NOT break the loop, so every failing
// step is documented and the drone gets its full step budget to recover.
types::MissionRunResult MissionControlImpl::runMission() {
    bool        completed = false;
    std::size_t steps     = 0;
    std::vector<types::ErrorRef> errors;

    try {
        for (std::size_t i = 0; i < mission_.max_steps; ++i) {
            const types::DroneStepResult step_result = drone_control_.step();
            ++steps;

            if (step_result.status == types::DroneStepStatus::Completed) {
                completed = true;
                break;
            }
            if (step_result.status == types::DroneStepStatus::Error) {
                errors.push_back({"DRONE_ERROR", step_result.message});
                // A real collision (drone actually inside an obstacle) is fatal: stop the mission
                // immediately so it does not spin in place. Any OTHER error is recorded
                // but does NOT break, so every failing step is documented and the drone keeps its full
                // step budget to recover (unchanged behaviour).
                if (step_result.message == "DRONE_COLLISION") {
                    break;
                }
            }
            // DroneStepStatus::Continue — proceed to the next step.
        }
    } catch (const std::exception& e) {
        // Any unexpected exception becomes an error entry; never propagate out of runMission (EH-1).
        errors.push_back({"EXCEPTION", e.what()});
    }

    types::MissionRunStatus status;
    if (completed) {
        status = types::MissionRunStatus::Completed;
    } else if (!errors.empty()) {
        status = types::MissionRunStatus::Error;
    } else {
        status = types::MissionRunStatus::MaxSteps;
    }

    // Always save, even on failure. A throw here propagates to SimulationRunImpl::run(), which has
    // its own try-catch (EH-1 satisfied at that layer).
    output_map_.save(output_map_file_);

    return {status, steps, std::move(errors)};
}

} // namespace drone_mapper
