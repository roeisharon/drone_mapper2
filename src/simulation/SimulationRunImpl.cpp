#include <drone_mapper/simulation/SimulationRunImpl.h>

#include <drone_mapper/utils/ErrorLogger.h>
#include <drone_mapper/utils/MapsComparison.h>

#include <exception>
#include <filesystem>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

// Drives one simulation run end to end:
//   1. runMission() (its exceptions isolated so a mission crash still yields a scored result),
//   2. resolution-request status (base build: always Accepted — output resolution equals the input
//      map's; output_mapping_resolution_factor handling is a bonus, so no request is ever rejected),
//   3. score the output map against the hidden map (skipped on mission Error → sentinel -1),
//   4. assemble and return the SimulationResult.
// The error log lives beside the output map (same stem, .log extension) so a human finds both in the
// same output_results/ directory. ErrorLogger creates the file lazily — nothing is written unless an
// error is actually logged.
types::SimulationResult SimulationRunImpl::run() {
    const std::filesystem::path error_log_path =
        output_map_file_.parent_path() / (output_map_file_.stem().string() + ".log");
    ErrorLogger logger{error_log_path};

    try {
        types::MissionRunResult mission_result;
        try {
            mission_result = mission_control_->runMission();
        } catch (const std::exception& e) {
            // A mission crash is logged at the point it occurs and converted into an error-scored
            // result rather than propagating and leaving the run half-assembled.
            try { logger.log("MISSION_EXCEPTION", e.what()); } catch (...) {}
            return types::SimulationResult{
                simulation_config_,
                mission_config_,
                types::ResolutionRequestStatus::Accepted,
                {types::MissionRunResult{
                    types::MissionRunStatus::Error, 0,
                    {{"MISSION_EXCEPTION", e.what()}}}},
                output_map_file_,
                output_map_->getMapConfig(),
                -1.0
            };
        }

        // Log every drone-level error accumulated during the step loop (the mission still reached a
        // terminal status, so these are non-fatal and do not abort scoring).
        for (const auto& err : mission_result.errors) {
            logger.log(err.code, err.message);
        }

        // Base build: input and output map resolutions are identical, so the resolution request is
        // always Accepted (factor-based output sizing is a bonus feature). See CLAUDE.md.
        const types::ResolutionRequestStatus res_status = types::ResolutionRequestStatus::Accepted;

        // A mission Error means the output map may be in an unknown state — skip comparison and
        // return the sentinel -1. Completed and MaxSteps both stopped cleanly, so the map is worth
        // scoring even if mapping was incomplete.
        double score = -1.0;
        if (mission_result.status != types::MissionRunStatus::Error) {
            const auto scores = MapsComparison::compare(*hidden_map_, {output_map_.get()});
            score = scores.empty() ? -1.0 : scores[0];
        }

        return types::SimulationResult{
            simulation_config_,
            mission_config_,
            res_status,
            {mission_result},
            output_map_file_,
            output_map_->getMapConfig(),
            score
        };

    } catch (const std::exception&) {
        // Backstop for unexpected exceptions from logger I/O or post-mission code: return a minimal
        // error result (score -1) rather than letting the process crash.
        return types::SimulationResult{
            simulation_config_,
            mission_config_,
            types::ResolutionRequestStatus::Accepted,
            {},
            output_map_file_,
            {},
            -1.0
        };
    }
}

} // namespace drone_mapper
