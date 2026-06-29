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

// Builds a failed-scenario SimulationResult: score -1 (the error sentinel) carrying the configs and,
// when known, an error code + descriptive message so the report shows WHY the scenario failed.
types::SimulationResult errorResult(const types::SimulationConfigData& simulation,
                                    const types::MissionConfigData& mission,
                                    const std::string& code,
                                    const std::string& message) {
    types::SimulationResult r;
    r.simulation_config = simulation;
    r.mission_config    = mission;
    r.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    r.mission_score = -1.0; // sentinel: this scenario errored and was not scored
    if (!code.empty()) {
        r.mission_results.push_back(types::MissionRunResult{
            types::MissionRunStatus::Error, 0, {{code, message}}});
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
    // Failure handling (both per the spec):
    //   - A single scenario that fails is scored -1 with its error code and the loop CONTINUES to the
    //     next scenario (a mission that errors mid-run already returns a -1 SimulationResult from
    //     SimulationRunImpl; the manager simply keeps going).
    //   - When an entire group cannot run — e.g. its map file fails to load, which the factory hits
    //     for every combination sharing that simulation — the first failure marks the whole group
    //     failed and every remaining combination is auto-filled with score -1 and the SAME error code,
    //     without calling the factory again. Other groups are unaffected.
    //   - A mission whose boundaries are inverted fails all of its own combinations with
    //     MISSION_BOUNDARY_INVALID (steps 0, score -1) and never calls the factory.
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        bool        group_failed = false;
        std::string group_code;    // error code shared by every auto-filled scenario in a dead group
        std::string group_message; // human-readable cause (e.g. the map-load error text)

        for (const types::MissionConfigData& mission : missions) {
            const bool bounds_invalid = missionBoundsInverted(mission.mission_bounds);
            if (bounds_invalid) {
                try { logger.log("MISSION_BOUNDARY_INVALID", "mission boundaries are inverted"); }
                catch (...) {}
            }

            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    if (group_failed) {
                        runs.push_back(errorResult(simulation, mission, group_code, group_message));
                        continue;
                    }
                    if (bounds_invalid) {
                        runs.push_back(errorResult(simulation, mission, "MISSION_BOUNDARY_INVALID",
                                                   "mission boundaries are inverted"));
                        continue;
                    }
                    try {
                        auto run_ptr = run_factory_->create(simulation, mission, drone, lidar, output_path);
                        runs.push_back(run_ptr->run());
                    } catch (const std::exception& e) {
                        // Run setup failed (commonly a bad/missing map file). Treat as group-wide:
                        // record the code now and reuse it for the rest of the group.
                        group_failed  = true;
                        group_code    = "FACTORY_ERROR";
                        group_message = e.what();
                        try { logger.log(group_code, group_message); } catch (...) {}
                        runs.push_back(errorResult(simulation, mission, group_code, group_message));
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
