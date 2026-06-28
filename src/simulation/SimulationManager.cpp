#include <drone_mapper/simulation/SimulationManager.h>

#include <drone_mapper/utils/ErrorLogger.h>
#include <drone_mapper/utils/SimulationReportWriter.h>

#include <chrono>
#include <ctime>
#include <exception>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

namespace {

// Current wall-clock time as "YYYY-MM-DDTHH:MM:SSZ".
std::string utcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_utc{};
    gmtime_r(&t, &tm_utc);
    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

// Mission boundaries are invalid when any axis is inverted (min > max). An absent block parses to a
// default all-zero MappingBounds (min == max), which is NOT inverted, so absent bounds are valid.
bool missionBoundsInverted(const types::MappingBounds& b) {
    return b.min_x > b.max_x || b.min_y > b.max_y || b.min_height > b.max_height;
}

// Builds an error SimulationResult (score -1) carrying the configs and an optional error code.
types::SimulationResult errorResult(const types::SimulationConfigData& simulation,
                                    const types::MissionConfigData& mission,
                                    const std::string& code) {
    types::SimulationResult r;
    r.simulation_config = simulation;
    r.mission_config    = mission;
    r.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    r.mission_score = -1.0;
    if (!code.empty()) {
        r.mission_results.push_back(types::MissionRunResult{
            types::MissionRunStatus::Error, 0, {{code, code}}});
    }
    return r;
}

} // namespace

types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);

    const std::string utc = utcTimestamp();
    std::vector<types::SimulationResult> runs;

    // Factory/boundary failures are logged immediately; the file is created lazily on first log().
    ErrorLogger logger{output_path / "simulation_manager_errors.log"};

    // Cartesian product per simulation group: missions × drones × lidars.
    // Group-failure: if the factory throws for a combination, every remaining combination in THAT
    // simulation group is filled with score -1 without calling the factory again. Other groups run
    // normally. A mission whose boundaries are invalid fails all its combinations with
    // MISSION_BOUNDARY_INVALID (steps 0, score -1) and never calls the factory.
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        bool group_failed = false;

        for (const types::MissionConfigData& mission : missions) {
            const bool bounds_invalid = missionBoundsInverted(mission.mission_bounds);
            if (bounds_invalid) {
                try { logger.log("MISSION_BOUNDARY_INVALID", "mission boundaries are inverted"); }
                catch (...) {}
            }

            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    if (group_failed) {
                        runs.push_back(errorResult(simulation, mission, ""));
                        continue;
                    }
                    if (bounds_invalid) {
                        runs.push_back(errorResult(simulation, mission, "MISSION_BOUNDARY_INVALID"));
                        continue;
                    }
                    try {
                        auto run_ptr = run_factory_->create(simulation, mission, drone, lidar, output_path);
                        runs.push_back(run_ptr->run());
                    } catch (const std::exception& e) {
                        group_failed = true;
                        try { logger.log("FACTORY_ERROR", e.what()); } catch (...) {}
                        runs.push_back(errorResult(simulation, mission, ""));
                    }
                }
            }
        }
    }

    types::SimulationManagerReport report{
        utc,                   // generated_at_utc
        "output_map_accuracy", // metric (fixed per spec)
        {0.0, 100.0},          // score_range (fixed per spec)
        -1,                    // error_score (fixed per spec)
        runs,
    };

    // Standalone write (no source paths → fallback grouping by map filename). main re-writes the
    // authoritative path-aware version using the parser's CompositionPaths.
    SimulationReportWriter::write(report, output_path, composition.composition_file);

    return report;
}

} // namespace drone_mapper
