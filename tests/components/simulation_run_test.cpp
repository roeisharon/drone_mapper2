#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "fixtures/SimulationRunFixture.h"

#include <drone_mapper/simulation/SimulationRunFactoryImpl.h>
#include <drone_mapper/Units.h>

#include <stdexcept>

// The SimulationRun fixture and its helpers (singleVoxelConfig, completedResult, maxStepsResult,
// errorResult, makeRun) live in tests/fixtures/SimulationRunFixture.h. All tests use TEST_F with
// that shared fixture so --gtest_filter=SimulationRun.* covers this file and mock_gps_movement_test.cpp.

using namespace ::testing;
using drone_mapper::types::MissionRunStatus;
using drone_mapper::types::MissionRunResult;
using drone_mapper::types::SimulationConfigData;
using drone_mapper::types::ResolutionRequestStatus;
using drone_mapper::types::VoxelOccupancy;
using drone_mapper::cm;

// Mission control delegation

// run() delegates to mission_control exactly once — no retry, no skip.
TEST_F(SimulationRun, RunCallsRunMissionExactlyOnce) {
    auto run = makeRun();
    EXPECT_CALL(*mock_mission_, runMission()).Times(1);
    (void)run->run();
}

// SimulationResult metadata fields

// The SimulationConfigData passed at construction appears verbatim in the result.
TEST_F(SimulationRun, RunReturnsSimulationConfig) {
    SimulationConfigData cfg;
    cfg.map_filename = "specific_building.npy";
    auto run = makeRun(cfg, mission_cfg_);
    const auto result = run->run();
    EXPECT_EQ(result.simulation_config.map_filename, "specific_building.npy");
}

// The MissionConfigData passed at construction appears verbatim in the result.
TEST_F(SimulationRun, RunReturnsMissionConfig) {
    mission_cfg_.max_steps = 42;
    auto run = makeRun(sim_cfg_, mission_cfg_);
    const auto result = run->run();
    EXPECT_EQ(result.mission_config.max_steps, 42u);
}

// The output file path supplied at construction is echoed back in the result.
TEST_F(SimulationRun, RunReturnsOutputMapFilePath) {
    const std::filesystem::path expected{"/tmp/specific_test_output.npy"};
    auto run = makeRun(sim_cfg_, mission_cfg_, expected);
    const auto result = run->run();
    EXPECT_EQ(result.output_map_file, expected);
}

// The output map's MapConfig is carried in the result so the manager can log it without touching
// the map object directly.
TEST_F(SimulationRun, RunReturnsOutputMapConfig) {
    const auto expected_cfg = singleVoxelConfig();
    auto run = makeRun();
    ON_CALL(*mock_output_, getMapConfig()).WillByDefault(Return(expected_cfg));
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.output_map_config.resolution.numerical_value_in(cm),
                     expected_cfg.resolution.numerical_value_in(cm));
}

// mission_results vector

// One simulation run drives one mission → exactly one entry in mission_results.
TEST_F(SimulationRun, RunContainsExactlyOneMissionResult) {
    auto run = makeRun();
    const auto result = run->run();
    EXPECT_EQ(result.mission_results.size(), 1u);
}

// A Completed mission status is preserved into the wrapped MissionRunResult.
TEST_F(SimulationRun, MissionStatusPropagatedCompleted) {
    auto run = makeRun();
    ON_CALL(*mock_mission_, runMission()).WillByDefault(Return(completedResult()));
    const auto result = run->run();
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, MissionRunStatus::Completed);
}

// A MaxSteps mission status is preserved unchanged.
TEST_F(SimulationRun, MissionStatusPropagatedMaxSteps) {
    auto run = makeRun();
    ON_CALL(*mock_mission_, runMission()).WillByDefault(Return(maxStepsResult()));
    const auto result = run->run();
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, MissionRunStatus::MaxSteps);
}

// Error entries (code + message) survive the trip through run() intact.
TEST_F(SimulationRun, MissionErrorsPreserved) {
    auto run = makeRun();
    const MissionRunResult err_res{MissionRunStatus::Error, 2, {{"ERR_CODE", "some_failure"}}};
    ON_CALL(*mock_mission_, runMission()).WillByDefault(Return(err_res));
    const auto result = run->run();
    ASSERT_EQ(result.mission_results.size(), 1u);
    ASSERT_EQ(result.mission_results[0].errors.size(), 1u);
    EXPECT_EQ(result.mission_results[0].errors[0].code, "ERR_CODE");
    EXPECT_EQ(result.mission_results[0].errors[0].message, "some_failure");
}

// Score computation

// A drone-level Error means the output map is unreliable → score is the sentinel -1.
TEST_F(SimulationRun, ScoreIsNegativeOneWhenMissionError) {
    auto run = makeRun();
    ON_CALL(*mock_mission_, runMission()).WillByDefault(Return(errorResult()));
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
}

// Completed → comparison runs. Hidden map all-Unmapped → no mappable voxels → score 0.0 (not -1,
// which would mean comparison was skipped).
TEST_F(SimulationRun, ScoreComputedWhenCompleted) {
    auto run = makeRun();
    ON_CALL(*mock_mission_, runMission()).WillByDefault(Return(completedResult()));
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, 0.0);
}

// MaxSteps is not an error; comparison still runs and yields a real (non-sentinel) score.
TEST_F(SimulationRun, ScoreComputedWhenMaxSteps) {
    auto run = makeRun();
    ON_CALL(*mock_mission_, runMission()).WillByDefault(Return(maxStepsResult()));
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, 0.0);
}

// Every mappable hidden voxel reproduced in the output → score 100.
TEST_F(SimulationRun, ScoreIs100WhenMapsIdentical) {
    auto run = makeRun();
    ON_CALL(*mock_hidden_, getMapConfig()).WillByDefault(Return(singleVoxelConfig()));
    ON_CALL(*mock_hidden_, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Empty));
    ON_CALL(*mock_output_, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Empty));
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
}

// Hidden has a mappable voxel but the output was never written (all Unmapped) → score 0.
TEST_F(SimulationRun, ScoreIsZeroWhenOutputUnmapped) {
    auto run = makeRun();
    ON_CALL(*mock_hidden_, getMapConfig()).WillByDefault(Return(singleVoxelConfig()));
    ON_CALL(*mock_hidden_, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Empty));
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, 0.0);
}

// An uncaught exception from runMission() is caught, treated as a mission error, and yields score
// -1 rather than propagating out of run().
TEST_F(SimulationRun, ScoreIsNegativeOneWhenMissionThrows) {
    auto run = makeRun();
    ON_CALL(*mock_mission_, runMission())
        .WillByDefault(Throw(std::runtime_error("mission_exploded")));
    const auto result = run->run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
}

// Resolution request status (base build: always Accepted — see CLAUDE.md)

// Factor 0 (unconfigured) → Accepted.
TEST_F(SimulationRun, ResolutionAcceptedWhenFactorZero) {
    mission_cfg_.output_mapping_resolution_factor = 0.0;
    auto run = makeRun(sim_cfg_, mission_cfg_);
    const auto result = run->run();
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);
}

// Factor 1 (output resolution equals input) → Accepted.
TEST_F(SimulationRun, ResolutionAcceptedWhenFactorOne) {
    mission_cfg_.output_mapping_resolution_factor = 1.0;
    auto run = makeRun(sim_cfg_, mission_cfg_);
    const auto result = run->run();
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);
}

// Factor > 1 (coarser output) → Accepted.
TEST_F(SimulationRun, ResolutionAcceptedWhenFactorAboveOne) {
    mission_cfg_.output_mapping_resolution_factor = 2.0;
    auto run = makeRun(sim_cfg_, mission_cfg_);
    const auto result = run->run();
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);
}

// Factor in (0,1) is still Accepted in the base build — factor-based output sizing is a bonus, so no
// request is ever rejected (the IgnoredTooSmall value is reserved for the bonus track).
TEST_F(SimulationRun, ResolutionAcceptedWhenFractional) {
    mission_cfg_.output_mapping_resolution_factor = 0.5;
    auto run = makeRun(sim_cfg_, mission_cfg_);
    const auto result = run->run();
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);
}

// SimulationRunFactoryImpl::outputMapConfig — mission bounds narrow the output map (Stage 1)

namespace {
using drone_mapper::x_extent;
using drone_mapper::y_extent;
using drone_mapper::z_extent;
using drone_mapper::Position3D;
using drone_mapper::types::MapConfig;
using drone_mapper::types::MappingBounds;

// A hidden-map config spanning [0,50) on each axis at 10 cm/voxel, no offset.
MapConfig hidden0to50() {
    return MapConfig{
        MappingBounds{0.0 * x_extent[cm], 50.0 * x_extent[cm],
                      0.0 * y_extent[cm], 50.0 * y_extent[cm],
                      0.0 * z_extent[cm], 50.0 * z_extent[cm]},
        Position3D{},
        10.0 * cm};
}
} // namespace

// No mission bounds (all-zero default) → output map is byte-identical to the hidden extent.
TEST(SimulationRunFactory, OutputMapMatchesHiddenWhenNoMissionBounds) {
    const MapConfig hidden = hidden0to50();
    const MapConfig out = drone_mapper::SimulationRunFactoryImpl::outputMapConfig(hidden, MappingBounds{});
    EXPECT_DOUBLE_EQ(out.boundaries.min_x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(out.boundaries.max_x.numerical_value_in(cm), 50.0);
    EXPECT_DOUBLE_EQ(out.offset.x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(out.resolution.numerical_value_in(cm), 10.0);
}

// Offset alone (non-zero offset, no mission bounds): the offset only relocates the origin — size and
// world extent are unchanged. A 5x5x5 @ 10cm map at offset +20 spans [-20,30) and stays full size.
TEST(SimulationRunFactory, OffsetAlonePreservesFullSize) {
    const MapConfig hidden{
        MappingBounds{-20.0 * x_extent[cm], 30.0 * x_extent[cm],
                      -20.0 * y_extent[cm], 30.0 * y_extent[cm],
                      -20.0 * z_extent[cm], 30.0 * z_extent[cm]},
        Position3D{20.0 * x_extent[cm], 20.0 * y_extent[cm], 20.0 * z_extent[cm]},
        10.0 * cm};
    const MapConfig out = drone_mapper::SimulationRunFactoryImpl::outputMapConfig(hidden, MappingBounds{});
    EXPECT_DOUBLE_EQ(out.boundaries.min_x.numerical_value_in(cm), -20.0);
    EXPECT_DOUBLE_EQ(out.boundaries.max_x.numerical_value_in(cm), 30.0); // span 50cm = 5 voxels, unchanged
    EXPECT_DOUBLE_EQ(out.offset.x.numerical_value_in(cm), 20.0);         // origin shift preserved
}

// Mission bounds equal to the full map → unchanged extent, offset stays 0 (benchmark invariant).
TEST(SimulationRunFactory, FullMissionBoundsEqualHiddenExtent) {
    const MapConfig hidden = hidden0to50();
    const MappingBounds full{0.0 * x_extent[cm], 50.0 * x_extent[cm],
                             0.0 * y_extent[cm], 50.0 * y_extent[cm],
                             0.0 * z_extent[cm], 50.0 * z_extent[cm]};
    const MapConfig out = drone_mapper::SimulationRunFactoryImpl::outputMapConfig(hidden, full);
    EXPECT_DOUBLE_EQ(out.boundaries.min_x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(out.boundaries.max_x.numerical_value_in(cm), 50.0);
    EXPECT_DOUBLE_EQ(out.offset.x.numerical_value_in(cm), 0.0);
}

// Sub-map mission bounds → output map narrows to the bounds, offset shifts so index 0 = lower corner.
TEST(SimulationRunFactory, SubMissionBoundsNarrowOutputMap) {
    const MapConfig hidden = hidden0to50();
    const MappingBounds sub{10.0 * x_extent[cm], 40.0 * x_extent[cm],
                            10.0 * y_extent[cm], 40.0 * y_extent[cm],
                            10.0 * z_extent[cm], 40.0 * z_extent[cm]};
    const MapConfig out = drone_mapper::SimulationRunFactoryImpl::outputMapConfig(hidden, sub);
    EXPECT_DOUBLE_EQ(out.boundaries.min_x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(out.boundaries.max_x.numerical_value_in(cm), 40.0);
    EXPECT_DOUBLE_EQ(out.offset.x.numerical_value_in(cm), -10.0); // index 0 sits at world 10
    EXPECT_DOUBLE_EQ(out.boundaries.max_height.numerical_value_in(cm), 40.0);
}

// Mission bounds wider than the map are clipped to the map (intersection, never an expansion).
TEST(SimulationRunFactory, MissionBoundsClippedToHiddenExtent) {
    const MapConfig hidden = hidden0to50();
    const MappingBounds wide{-100.0 * x_extent[cm], 100.0 * x_extent[cm],
                             -100.0 * y_extent[cm], 100.0 * y_extent[cm],
                             -100.0 * z_extent[cm], 100.0 * z_extent[cm]};
    const MapConfig out = drone_mapper::SimulationRunFactoryImpl::outputMapConfig(hidden, wide);
    EXPECT_DOUBLE_EQ(out.boundaries.min_x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(out.boundaries.max_x.numerical_value_in(cm), 50.0);
}

// Offset participation: a 5x5x5 @ 10cm map with offset -20 spans world [20,70) (min = -offset).
// Intersecting with mission [-20,30) gives [20,30) — a single 10cm voxel. Proves the intersection is
// computed in world coordinates against the offset-derived extent (not assuming the map starts at 0).
TEST(SimulationRunFactory, NegativeOffsetExtentIntersectedInWorldCoords) {
    const MapConfig hidden{
        MappingBounds{20.0 * x_extent[cm], 70.0 * x_extent[cm],
                      20.0 * y_extent[cm], 70.0 * y_extent[cm],
                      20.0 * z_extent[cm], 70.0 * z_extent[cm]},
        Position3D{-20.0 * x_extent[cm], -20.0 * y_extent[cm], -20.0 * z_extent[cm]},
        10.0 * cm};
    const MappingBounds mission{-20.0 * x_extent[cm], 30.0 * x_extent[cm],
                                -20.0 * y_extent[cm], 30.0 * y_extent[cm],
                                -20.0 * z_extent[cm], 30.0 * z_extent[cm]};
    const MapConfig out = drone_mapper::SimulationRunFactoryImpl::outputMapConfig(hidden, mission);
    EXPECT_DOUBLE_EQ(out.boundaries.min_x.numerical_value_in(cm), 20.0); // max(20,-20)
    EXPECT_DOUBLE_EQ(out.boundaries.max_x.numerical_value_in(cm), 30.0); // min(70,30)
    EXPECT_DOUBLE_EQ(out.offset.x.numerical_value_in(cm), -20.0);        // index 0 sits at world 20
}

// Offset +20 puts the same 5x5x5 map at world [-20,30); mission [-20,30) then covers it fully, so the
// output is the whole 5x5x5 map (span 50cm). This is the configuration that yields a full-map output.
TEST(SimulationRunFactory, PositiveOffsetFullMissionCoversWholeMap) {
    const MapConfig hidden{
        MappingBounds{-20.0 * x_extent[cm], 30.0 * x_extent[cm],
                      -20.0 * y_extent[cm], 30.0 * y_extent[cm],
                      -20.0 * z_extent[cm], 30.0 * z_extent[cm]},
        Position3D{20.0 * x_extent[cm], 20.0 * y_extent[cm], 20.0 * z_extent[cm]},
        10.0 * cm};
    const MappingBounds mission{-20.0 * x_extent[cm], 30.0 * x_extent[cm],
                                -20.0 * y_extent[cm], 30.0 * y_extent[cm],
                                -20.0 * z_extent[cm], 30.0 * z_extent[cm]};
    const MapConfig out = drone_mapper::SimulationRunFactoryImpl::outputMapConfig(hidden, mission);
    EXPECT_DOUBLE_EQ(out.boundaries.min_x.numerical_value_in(cm), -20.0);
    EXPECT_DOUBLE_EQ(out.boundaries.max_x.numerical_value_in(cm), 30.0); // full 50cm span = 5 voxels
    EXPECT_DOUBLE_EQ(out.offset.x.numerical_value_in(cm), 20.0);
}
