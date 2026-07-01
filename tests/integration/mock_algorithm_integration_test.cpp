#include <gtest/gtest.h>

#include "integration_support.h"

#include <drone_mapper/simulation/SimulationManager.h>
#include <drone_mapper/simulation/SimulationRunFactoryImpl.h>
#include <drone_mapper/simulation/SimulationRunImpl.h>
#include <drone_mapper/mission/MissionControlImpl.h>
#include <drone_mapper/drone/DroneControlImpl.h>
#include <drone_mapper/drone/IMappingAlgorithm.h>
#include <drone_mapper/mocks/MockGPS.h>
#include <drone_mapper/mocks/MockLidar.h>
#include <drone_mapper/mocks/MockMovement.h>
#include <drone_mapper/utils/SimulationReportWriter.h>

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

// Mock-algorithm end-to-end integration test (suite "Integration"). It runs the full flow through
// SimulationManager but injects a DETERMINISTIC scripted algorithm via a test factory that wires the
// REAL map / GPS / movement / lidar / scan writer / comparison / report — so the pipeline is exercised
// fast and reproducibly without the real planner. Shared helpers come from integration_support.h.

namespace {

// Emits a fixed number of forward scans (no movement) then Finished. Because it never moves, every run
// maps the same region deterministically; MockLidar still traces the real hidden map.
class ScriptedAlgorithm final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    types::MappingStepCommand nextStep(const types::DroneState& /*state*/,
                                       const types::LidarScanResult* /*latest_scan*/) override {
        if (calls_++ < kScanSteps) {
            return {std::nullopt,
                    Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                    types::AlgorithmStatus::Working};
        }
        return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
    }

private:
    static constexpr int kScanSteps = 8;
    int calls_ = 0;
};

// Replicates the production wiring (reusing the real static helpers) but swaps in ScriptedAlgorithm.
class ScriptedAlgoRunFactory final : public ISimulationRunFactory {
public:
    std::unique_ptr<ISimulationRun> create(const types::SimulationConfigData& simulation,
                                           const types::MissionConfigData& mission,
                                           const types::DroneConfigData& drone,
                                           const types::LidarConfigData& lidar,
                                           const fs::path& output_path) override {
        auto hidden = std::make_unique<Map3DImpl>(
            loadNpy(simulation.map_filename),
            types::MapConfig{types::MappingBounds{}, simulation.map_offset, simulation.map_resolution});

        const types::MapConfig output_cfg =
            SimulationRunFactoryImpl::outputMapConfig(hidden->getMapConfig(), mission.mission_bounds);
        auto output = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), output_cfg);

        auto gps = std::make_unique<MockGPS>(
            SimulationRunFactoryImpl::snapToCellCenter(simulation.initial_drone_position, output_cfg),
            Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
            mission.gps_resolution);
        auto movement = std::make_unique<MockMovement>(*gps, *hidden, drone.radius);
        auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden, *gps);
        auto algorithm = std::make_unique<ScriptedAlgorithm>(mission, lidar, drone, *output);
        auto control = std::make_unique<DroneControlImpl>(
            drone, mission, *lidar_impl, *gps, *movement, *output, *algorithm);

        const fs::path dir = output_path / "output_results";
        fs::create_directories(dir);
        const fs::path out_file =
            dir / (simulation.map_filename.stem().string() + "_mock_run" +
                   std::to_string(counter_.fetch_add(1)) + ".npy");
        auto mission_control = std::make_unique<MissionControlImpl>(
            mission, drone, *hidden, *output, *control, out_file);

        return std::make_unique<SimulationRunImpl>(
            std::move(hidden), std::move(output), std::move(gps), std::move(movement),
            std::move(lidar_impl), std::move(algorithm), std::move(control),
            std::move(mission_control), simulation, mission, out_file);
    }

private:
    std::atomic<std::size_t> counter_{0};
};

} // namespace

// ================================================================================================
// Mock (scripted) algorithm — full flow through the manager, with report + failure isolation.
// ================================================================================================
TEST(Integration, MockAlgorithmFullFlowThroughManagerAndReport) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_small.npy"))
        << "run the test binary from the repo root (inputs/ must be reachable)";

    const fs::path out = freshOutputDir("mock");
    ErrorLogger logger{out / "output_results" / "parse.log"};
    const SmallRoomInputs in = parseSmallRoom(logger);

    // One healthy group (2 lidars ⇒ 2 runs) plus a group whose map file is missing (2 runs that must
    // fail) — proves the manager continues through every combination and isolates failures.
    types::SimulationConfigData bad_sim = in.simulation;
    bad_sim.map_filename = kInputs / "map" / "NO_SUCH_MAP.npy";

    types::SimulationCompositionData comp;
    comp.composition_file = "integration_mock_compose.yaml";
    comp.simulation_mission_groups.emplace_back(
        in.simulation, std::vector<types::MissionConfigData>{in.mission});
    comp.simulation_mission_groups.emplace_back(
        bad_sim, std::vector<types::MissionConfigData>{in.mission});
    comp.drones = {in.drone_small};
    comp.lidars = {in.lidar_short, in.lidar_long};

    SimulationManager manager{std::make_unique<ScriptedAlgoRunFactory>()};
    const types::SimulationManagerReport report = manager.run(comp, out);

    // 2 groups × 1 mission × 1 drone × 2 lidars = 4 combinations, all present.
    ASSERT_EQ(report.runs.size(), 4u);
    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_EQ(report.error_score, -1);
    EXPECT_FALSE(report.generated_at_utc.empty());

    // Healthy group (runs 0,1): scripted algo → Completed, scored in range, output saved.
    for (int i : {0, 1}) {
        const types::SimulationResult& r = report.runs[i];
        ASSERT_FALSE(r.mission_results.empty());
        EXPECT_EQ(r.mission_results[0].status, types::MissionRunStatus::Completed);
        EXPECT_GE(r.mission_score, 0.0);
        EXPECT_LE(r.mission_score, 100.0);
        EXPECT_TRUE(fs::exists(r.output_map_file));
        EXPECT_GT(fs::file_size(r.output_map_file), 0u);
    }

    // Failed group (runs 2,3): scored -1, and crucially the healthy runs still ran.
    EXPECT_DOUBLE_EQ(report.runs[2].mission_score, -1.0);
    EXPECT_DOUBLE_EQ(report.runs[3].mission_score, -1.0);

    // YAML report written with valid, present fields (manager writes it; re-write to assert explicitly).
    SimulationReportWriter::write(report, out, comp.composition_file);
    const fs::path yaml = out / "simulation_output.yaml";
    ASSERT_TRUE(fs::exists(yaml));
    EXPECT_GT(fs::file_size(yaml), 0u);
    const std::string text = readFile(yaml);
    EXPECT_NE(text.find("output_map_accuracy"), std::string::npos);
    EXPECT_NE(text.find("score"), std::string::npos);
    EXPECT_NE(text.find("-1"), std::string::npos) << "the failed runs' -1 sentinel must appear";
}
