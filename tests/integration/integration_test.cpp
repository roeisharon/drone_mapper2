#include <gtest/gtest.h>

#include "integration_support.h"

#include <drone_mapper/simulation/SimulationManager.h>
#include <drone_mapper/simulation/SimulationRunFactoryImpl.h>

#include <memory>
#include <vector>

// Real-algorithm end-to-end integration tests (suite "Integration"). These drive the WHOLE pipeline —
// YAML parsing → SimulationManager → real SimulationRunFactoryImpl → SimulationRun → MissionControl →
// DroneControl → Map3D/MockLidar/MockMovement/MockGPS → ScanResultToVoxels → MapsComparison →
// SimulationReportWriter — over the course-provided inputs/ scenarios (the same maps/configs used to
// develop and debug the project; scenario_house.npy IS the benchmark map). Assertions are on concrete
// observable behaviour so an injected bug in almost any component perturbs at least one of them.
//
// The mock-algorithm full-flow test lives in mock_algorithm_integration_test.cpp; both share
// integration_support.h.

// ================================================================================================
// 1. Real algorithm, real components — full flow through SimulationManager over course inputs.
// ================================================================================================
TEST(Integration, RealAlgorithmFullFlowOverCourseInputs) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_small.npy"))
        << "run the test binary from the repo root (inputs/ must be reachable)";

    const fs::path out = freshOutputDir("real");
    ErrorLogger logger{out / "output_results" / "parse.log"};
    const SmallRoomInputs in = parseSmallRoom(logger);

    // Config parsing populated real values (guards the YAML parser + diameter→radius).
    ASSERT_GT(in.mission.max_steps, 0u);
    ASSERT_GT(in.drone_small.radius.numerical_value_in(cm), 0.0);
    ASSERT_GT(in.drone_large.radius.numerical_value_in(cm),
              in.drone_small.radius.numerical_value_in(cm));
    ASSERT_TRUE(fs::exists(in.simulation.map_filename)) << in.simulation.map_filename;

    types::SimulationCompositionData comp;
    comp.composition_file = "integration_real_compose.yaml";
    comp.simulation_mission_groups.emplace_back(
        in.simulation, std::vector<types::MissionConfigData>{in.mission});
    comp.drones = {in.drone_small, in.drone_large};
    comp.lidars = {in.lidar_short};

    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>()};
    const types::SimulationManagerReport report = manager.run(comp, out);

    // Cartesian order is group→mission→drone→lidar, so runs[0]=small×short, runs[1]=large×short.
    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);

    const types::SimulationResult& small_run = report.runs[0];
    const types::SimulationResult& large_run = report.runs[1];

    // Completes successfully, no error status.
    ASSERT_FALSE(small_run.mission_results.empty());
    EXPECT_EQ(small_run.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_TRUE(small_run.mission_results[0].errors.empty());

    // Output .npy created and non-empty on disk.
    ASSERT_TRUE(fs::exists(small_run.output_map_file));
    EXPECT_GT(fs::file_size(small_run.output_map_file), 0u);

    // Output shape equals the mission-bounds region: x[0,200) y[90,200) z[0,90) @ 10cm ⇒ 20×11×9.
    const MapStats stats = censusOutputMap(small_run.output_map_file, small_run.output_map_config);
    EXPECT_EQ(stats.nx, 20);
    EXPECT_EQ(stats.ny, 11);
    EXPECT_EQ(stats.nz, 9);

    // Meaningful mapped content, not only Unmapped (catches lidar/scan-writer/movement/algo failures).
    EXPECT_GT(stats.mapped, 100) << "the drone should have observed a large part of the room";
    EXPECT_GT(stats.empty, 0);
    EXPECT_GT(stats.occupied, 0) << "walls of the room must be mapped as Occupied";

    // Score is a real value in range and high for a drone that fits the room.
    EXPECT_GE(small_run.mission_score, 0.0);
    EXPECT_LE(small_run.mission_score, 100.0);
    EXPECT_GT(small_run.mission_score, 50.0);

    // Different drone sizes → different coverage: the 15cm drone is wedged at the wall/floor start and
    // maps almost nothing, so it scores strictly below the small drone.
    ASSERT_FALSE(large_run.mission_results.empty());
    EXPECT_EQ(large_run.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_GT(small_run.mission_score, large_run.mission_score);

    // The manager wrote the report file.
    EXPECT_TRUE(fs::exists(out / "simulation_output.yaml"));
}

// ================================================================================================
// 2. Real NPY load path normalizes Minecraft-style multi-valued blocks to occupancy.
// ================================================================================================
TEST(Integration, RealNpyLoadNormalizesMinecraftBlocksToOccupancy) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_house.npy"));
    // scenario_house.npy IS the benchmark map; it holds block ids {0,1,2,3,4,18,45}.
    const types::MapConfig cfg{types::MappingBounds{}, Position3D{}, 10.0 * cm};
    Map3DImpl map{loadNpy(kInputs / "map" / "scenario_house.npy"), cfg};

    // Array index (0,0,0) holds block id 3 — a solid, non-1 block. It MUST normalize to Occupied,
    // not be mistaken for a special sentinel byte.
    EXPECT_EQ(map.atVoxel(worldCm(5, 5, 5)), types::VoxelOccupancy::Occupied);
    // Index (14,17,21) is air (0) → Empty.
    EXPECT_EQ(map.atVoxel(worldCm(145, 175, 215)), types::VoxelOccupancy::Empty);
    // A world position outside the array → OutOfBounds (not silently Empty/Occupied).
    EXPECT_EQ(map.atVoxel(worldCm(-50, 0, 0)), types::VoxelOccupancy::OutOfBounds);
}

// ================================================================================================
// 3. Map offset shifts world boundaries: min = -offset, max = shape*res - offset.
// ================================================================================================
TEST(Integration, MapOffsetShiftsWorldBoundaries) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_house.npy"));
    // scenario_house is (29,30,31); apply a height offset of 150cm (the house scenario's own offset).
    const types::MapConfig cfg{types::MappingBounds{}, worldCm(0, 0, 150), 10.0 * cm};
    Map3DImpl map{loadNpy(kInputs / "map" / "scenario_house.npy"), cfg};
    const types::MappingBounds b = map.getMapConfig().boundaries;

    EXPECT_DOUBLE_EQ(b.min_height.numerical_value_in(cm), -150.0);
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), 31 * 10.0 - 150.0);
    EXPECT_DOUBLE_EQ(b.min_x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(cm), 29 * 10.0);
}

// ================================================================================================
// 4. Start-position validation fails one scenario with -1 without affecting the flow.
// ================================================================================================
TEST(Integration, StartOutsideMapFailsScenarioWithNegativeOne) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_small.npy"));

    const fs::path out = freshOutputDir("startval");
    ErrorLogger logger{out / "output_results" / "parse.log"};
    const SmallRoomInputs in = parseSmallRoom(logger);

    // Move the start far outside the map extent; SimulationRunImpl::run must reject it before running.
    types::SimulationConfigData sim = in.simulation;
    sim.initial_drone_position = worldCm(100000, 100000, 100000);

    types::SimulationCompositionData comp;
    comp.composition_file = "integration_startval_compose.yaml";
    comp.simulation_mission_groups.emplace_back(
        sim, std::vector<types::MissionConfigData>{in.mission});
    comp.drones = {in.drone_small};
    comp.lidars = {in.lidar_short};

    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>()};
    const types::SimulationManagerReport report = manager.run(comp, out);

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error);
    EXPECT_EQ(report.runs[0].mission_results[0].steps, 0u);
    ASSERT_FALSE(report.runs[0].mission_results[0].errors.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].code, "START_OUTSIDE_MAP");
}
