#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "fixtures/SimulationRunFixture.h"

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
