#include <gtest/gtest.h>

#include "integration_support.h"

#include <drone_mapper/simulation/SimulationManager.h>
#include <drone_mapper/simulation/SimulationRunFactoryImpl.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// Real-algorithm end-to-end integration tests (suite "Integration"). These drive the WHOLE pipeline —
// YAML parsing → SimulationManager → real SimulationRunFactoryImpl → SimulationRun → MissionControl →
// DroneControl → Map3D/MockLidar/MockMovement/MockGPS → ScanResultToVoxels → MapsComparison →
// SimulationReportWriter — over the course-provided inputs/ scenarios. scenario_house.npy is the
// benchmark-style staff map (formerly benchmark_map) used to validate Checkpoints A/B/C. Assertions
// are on concrete observable behaviour so an injected bug in almost any component perturbs one of them.
//
// The mock-algorithm full-flow test lives in mock_algorithm_integration_test.cpp; both share
// integration_support.h.

namespace {

// Hand-rolled configs for the tiny 5x5x5 data_maps fixture (there is no inputs/ YAML for it).
types::SimulationConfigData simpleMapSim() {
    // single_voxel_x4_y4_z4.npy: 5x5x5, one Occupied voxel at (4,4,4), 124 Empty. Start at the (0,0,0)
    // cell centre (0.5cm) so the radius-0.5 footprint only touches the map edge, never penetrates it.
    return types::SimulationConfigData{
        "data_maps/single_voxel_x4_y4_z4.npy",
        1.0 * cm,
        Position3D{},
        worldCm(0.5, 0.5, 0.5),
        0.0 * horizontal_angle[deg],
    };
}

types::DroneConfigData simpleSmallDrone() {
    return types::DroneConfigData{0.5 * cm, 45.0 * horizontal_angle[deg], 1.0 * cm, 1.0 * cm};
}

types::LidarConfigData simpleFastLidar() {
    // fov_circles=2, short range — cheap to trace on the tiny map.
    return types::LidarConfigData{0.5 * cm, 2.0 * cm, 0.5 * cm, 2};
}

types::MissionConfigData simpleMission() {
    return types::MissionConfigData{2000, 1.0 * cm, 1.0, types::MappingBounds{}};
}

int countNpyFiles(const fs::path& dir) {
    int n = 0;
    if (!fs::exists(dir)) {
        return 0;
    }
    for (const auto& entry : fs::recursive_directory_iterator(dir)) {
        if (entry.path().extension() == ".npy") {
            ++n;
        }
    }
    return n;
}

} // namespace

// ================================================================================================
// 1. Fast happy path on a trivial 5x5x5 map — the small drone should map essentially everything.
// ================================================================================================
TEST(Integration, SimpleMapHappyPathAchievesHighCoverage) {
    ASSERT_TRUE(fs::exists("data_maps/single_voxel_x4_y4_z4.npy"))
        << "run the test binary from the repo root";

    const fs::path out = freshOutputDir("simple");

    types::SimulationCompositionData comp;
    comp.composition_file = "integration_simple_compose.yaml";
    comp.simulation_mission_groups.emplace_back(
        simpleMapSim(), std::vector<types::MissionConfigData>{simpleMission()});
    comp.drones = {simpleSmallDrone()};
    comp.lidars = {simpleFastLidar()};

    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>()};
    const types::SimulationManagerReport report = manager.run(comp, out);

    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_NE(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error);
    EXPECT_TRUE(fs::exists(report.runs[0].output_map_file));
    // An unobstructed 5x5x5 room fully explored ⇒ near-100. A generous floor still fails hard on a
    // gross regression (stuck drone / broken scan / broken scoring ⇒ ~0) without being brittle.
    EXPECT_GE(report.runs[0].mission_score, 95.0)
        << "trivial map should map almost perfectly; got " << report.runs[0].mission_score;
    EXPECT_LE(report.runs[0].mission_score, 100.0);
}

// ================================================================================================
// 2. PRIMARY: full real-algorithm flow on the staff benchmark map (scenario_house) + runtime sanity.
// ================================================================================================
TEST(Integration, RealAlgorithmFullFlowOnHouseBenchmark) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_house.npy"))
        << "run the test binary from the repo root (inputs/ must be reachable)";

    const fs::path out = freshOutputDir("house");
    ErrorLogger logger{out / "output_results" / "parse.log"};

    // Use the FULL-RAW-MAP house scenario (offset 0, mission covering the whole array) so the output map
    // is the ENTIRE scenario_house extent, not just the offset-clipped house region. The large (15cm)
    // drone starts at voxel (4,4,15) — the drone-start gap on the house floor — and exercises footprint
    // handling / entrance fitting; it needs fewer steps than the small drone, keeping the run bounded.
    const auto sim     = config::parseSimulationConfig(kInputs / "simulation" / "house_simulation_full_raw_map.yaml", logger);
    const auto mission = config::parseMissionConfig(kInputs / "mission" / "house_mission_full_raw_map.yaml", logger);
    const auto drone   = config::parseDroneConfig(kInputs / "drone" / "drone_large.yaml", logger);
    const auto lidar   = config::parseLidarConfig(kInputs / "lidar" / "lidar_short.yaml", logger);
    ASSERT_TRUE(fs::exists(sim.map_filename)) << sim.map_filename;

    types::SimulationCompositionData comp;
    comp.composition_file = "integration_house_compose.yaml";
    comp.simulation_mission_groups.emplace_back(
        sim, std::vector<types::MissionConfigData>{mission});
    comp.drones = {drone};
    comp.lidars = {lidar};

    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>()};
    const auto t0 = std::chrono::steady_clock::now();
    const types::SimulationManagerReport report = manager.run(comp, out);
    const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
                               std::chrono::steady_clock::now() - t0)
                               .count();

    // Runtime sanity (idea 4): a bounded house run must not blow up.
    EXPECT_LT(elapsed_s, 60) << "house run took " << elapsed_s << "s (limit 60)";

    ASSERT_EQ(report.runs.size(), 1u);
    const types::SimulationResult& run = report.runs[0];

    // No error, actually completed exploration.
    ASSERT_FALSE(run.mission_results.empty());
    EXPECT_EQ(run.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_TRUE(run.mission_results[0].errors.empty());

    // Output map created, non-empty, and the shape is the ENTIRE scenario_house array. With offset 0 the
    // hidden extent is world x[0,290) y[0,300) z[0,310) and the mission covers all of it ⇒ 29×30×31.
    // (Array z 0–14 is solid "earth" below the drone-start floor at z-15; it is inside the output extent
    // but the drone maps the reachable house above it.)
    ASSERT_TRUE(fs::exists(run.output_map_file));
    EXPECT_GT(fs::file_size(run.output_map_file), 0u);
    const MapStats stats = censusOutputMap(run.output_map_file, run.output_map_config);
    EXPECT_EQ(stats.nx, 29);
    EXPECT_EQ(stats.ny, 30);
    EXPECT_EQ(stats.nz, 31);

    // Meaningful mapped content (block normalization + scanning + serialization all worked).
    EXPECT_GT(stats.mapped, 0);
    EXPECT_GT(stats.empty, 0);
    EXPECT_GT(stats.occupied, 0) << "house structure must be mapped as Occupied";

    // A sane, real score. The scored region now spans the whole map (the solid-earth floor is included),
    // so the exact value differs from the offset-clipped run; a conservative floor still fails hard on a
    // gross regression (stuck drone / broken scan / broken scoring ⇒ ~0) without being brittle to how
    // much of the earth floor gets scanned.
    EXPECT_GT(run.mission_score, 40.0) << "unexpectedly low score: " << run.mission_score;
    EXPECT_LE(run.mission_score, 100.0);
}

// ================================================================================================
// 3. Fast scenario_small run over MULTIPLE drone/lidar combinations: one output map each + drone-size
//    differentiation (kept small for runtime, per the "keep one fast scenario_small test" request).
// ================================================================================================
TEST(Integration, MultiCombinationOutputsAndDroneSizeDifferentiation) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_small.npy"));

    const fs::path out = freshOutputDir("multi");
    ErrorLogger logger{out / "output_results" / "parse.log"};
    const SmallRoomInputs in = parseSmallRoom(logger);

    types::SimulationCompositionData comp;
    comp.composition_file = "integration_multi_compose.yaml";
    comp.simulation_mission_groups.emplace_back(
        in.simulation, std::vector<types::MissionConfigData>{in.mission});
    comp.drones = {in.drone_small, in.drone_large};
    comp.lidars = {in.lidar_short, in.lidar_long};

    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>()};
    const types::SimulationManagerReport report = manager.run(comp, out);

    // 1 group × 1 mission × 2 drones × 2 lidars = 4 runs, order drone-outer then lidar-inner:
    // [small×short, small×long, large×short, large×long].
    ASSERT_EQ(report.runs.size(), 4u);
    for (const types::SimulationResult& r : report.runs) {
        ASSERT_FALSE(r.mission_results.empty());
        EXPECT_EQ(r.mission_results[0].status, types::MissionRunStatus::Completed);
    }

    // Each combination produced exactly one output .npy map.
    EXPECT_EQ(countNpyFiles(out / "output_results"), 4);

    // Drone-size differentiation: the small drone fits the room and maps it; the 15cm drone is wedged
    // at the wall/floor start and maps almost nothing, so both small runs beat both large runs.
    const double small_min = std::min(report.runs[0].mission_score, report.runs[1].mission_score);
    const double large_max = std::max(report.runs[2].mission_score, report.runs[3].mission_score);
    EXPECT_GT(small_min, large_max)
        << "small=[" << report.runs[0].mission_score << "," << report.runs[1].mission_score
        << "] large=[" << report.runs[2].mission_score << "," << report.runs[3].mission_score << "]";
}

// ================================================================================================
// 4. Real NPY load path normalizes Minecraft-style multi-valued blocks to occupancy.
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
// 5. Load derives world boundaries from the NPY shape (29×30×31) and offset: min=-offset,
//    max=shape*res-offset.
// ================================================================================================
TEST(Integration, MapLoadDerivesWorldBoundariesFromShapeAndOffset) {
    ASSERT_TRUE(fs::exists(kInputs / "map" / "scenario_house.npy"));
    // scenario_house is (29,30,31); apply a height offset of 150cm (the house scenario's own offset).
    const types::MapConfig cfg{types::MappingBounds{}, worldCm(0, 0, 150), 10.0 * cm};
    Map3DImpl map{loadNpy(kInputs / "map" / "scenario_house.npy"), cfg};
    const types::MappingBounds b = map.getMapConfig().boundaries;

    EXPECT_DOUBLE_EQ(b.min_x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(cm), 29 * 10.0);
    EXPECT_DOUBLE_EQ(b.min_y.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(b.max_y.numerical_value_in(cm), 30 * 10.0);
    EXPECT_DOUBLE_EQ(b.min_height.numerical_value_in(cm), -150.0);
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), 31 * 10.0 - 150.0);
}

// ================================================================================================
// 6. Start-position validation fails one scenario with -1 without affecting the flow.
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
