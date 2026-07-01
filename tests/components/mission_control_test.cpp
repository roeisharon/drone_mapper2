#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/mission/MissionControlImpl.h>

#include "mocks/MockIDroneControl.h"
#include "mocks/MockIMap3D.h"
#include "mocks/MockIMutableMap3D.h"

#include <filesystem>
#include <memory>
#include <string>

namespace {

using drone_mapper::cm;
using drone_mapper::types::DroneStepResult;
using drone_mapper::types::DroneStepStatus;
using drone_mapper::types::MissionRunStatus;
using drone_mapper::types::MissionConfigData;
using drone_mapper::types::DroneConfigData;

DroneStepResult continueStep()      { return {DroneStepStatus::Continue,  {}}; }
DroneStepResult realCollisionStep() { return {DroneStepStatus::Error, "DRONE_COLLISION"}; }
DroneStepResult completedStep()     { return {DroneStepStatus::Completed, {}}; }
DroneStepResult errorStep(const std::string& msg = "some_error") {
    return {DroneStepStatus::Error, msg};
}

MissionConfigData makeConfig(std::size_t max_steps) {
    MissionConfigData cfg;
    cfg.max_steps      = max_steps;
    cfg.gps_resolution = 10.0 * cm;
    return cfg;
}

const std::filesystem::path kDefaultPath{"/tmp/mission_ctrl_test_output.npy"};

} // namespace

// Fixture named "MissionControl" so --gtest_filter=MissionControl.* works; at global scope so it
// does not shadow drone_mapper::MissionControlImpl.
class MissionControl : public ::testing::Test {
protected:
    void SetUp() override {
        using namespace ::testing;
        ON_CALL(drone_control_, step()).WillByDefault(Return(continueStep())); // Continue forever
        ON_CALL(output_map_, save(_)).WillByDefault(Return());
    }

    std::unique_ptr<drone_mapper::MissionControlImpl>
    makeMission(std::size_t max_steps, const std::filesystem::path& path = kDefaultPath) {
        return std::make_unique<drone_mapper::MissionControlImpl>(
            makeConfig(max_steps), DroneConfigData{}, hidden_map_, output_map_, drone_control_, path);
    }

    ::testing::NiceMock<drone_mapper::mocks::MockIDroneControl> drone_control_;
    ::testing::NiceMock<drone_mapper::mocks::MockIMap3D>        hidden_map_;
    ::testing::NiceMock<drone_mapper::mocks::MockIMutableMap3D> output_map_;
};

// Termination status

// step() Completed → mission Completed.
TEST_F(MissionControl, StatusIsCompletedWhenStepReturnsCompleted) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(completedStep()));
    EXPECT_EQ(makeMission(10)->runMission().status, MissionRunStatus::Completed);
}

// Loop exhausts with only Continue → MaxSteps.
TEST_F(MissionControl, StatusIsMaxStepsWhenLoopExhaustsWithNoErrors) {
    EXPECT_EQ(makeMission(3)->runMission().status, MissionRunStatus::MaxSteps);
}

// A real collision (drone actually inside an obstacle → DRONE_COLLISION) is fatal and stops the
// mission on the FIRST occurrence (Checkpoint C): status Error, exactly one step taken, and the
// collision recorded — so SimulationRunImpl scores the scenario -1 and does not spin to max_steps.
TEST_F(MissionControl, RealCollisionStopsMissionImmediatelyWithError) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(realCollisionStep()));
    const auto result = makeMission(10)->runMission();
    EXPECT_EQ(result.status, MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1u) << "the mission must stop on the first real collision, not run on";
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].message, "DRONE_COLLISION");
}

// Errors accumulate and the loop exhausts without Completed → Error.
TEST_F(MissionControl, StatusIsErrorWhenStepReturnsErrorAndLoopExhausts) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(errorStep("motor_stall")));
    EXPECT_EQ(makeMission(5)->runMission().status, MissionRunStatus::Error);
}

// max_steps == 0 → loop body never runs → MaxSteps.
TEST_F(MissionControl, StatusIsMaxStepsWhenMaxStepsIsZero) {
    EXPECT_EQ(makeMission(0)->runMission().status, MissionRunStatus::MaxSteps);
}

// Completed takes priority over earlier errors.
TEST_F(MissionControl, StatusIsCompletedEvenWithEarlierErrors) {
    using namespace ::testing;
    EXPECT_CALL(drone_control_, step())
        .WillOnce(Return(errorStep("glitch")))
        .WillOnce(Return(continueStep()))
        .WillOnce(Return(completedStep()));
    EXPECT_EQ(makeMission(10)->runMission().status, MissionRunStatus::Completed);
}

// Step count

// Step count equals the position of the Completed step.
TEST_F(MissionControl, StepCountAfterCompletedEqualsPositionOfCompletedStep) {
    using namespace ::testing;
    EXPECT_CALL(drone_control_, step())
        .WillOnce(Return(continueStep()))
        .WillOnce(Return(continueStep()))
        .WillOnce(Return(completedStep()));
    EXPECT_EQ(makeMission(10)->runMission().steps, 3u);
}

// Step count equals max_steps when the loop exhausts.
TEST_F(MissionControl, StepCountEqualsMaxStepsWhenLoopExhausts) {
    EXPECT_EQ(makeMission(7)->runMission().steps, 7u);
}

// Step count is zero when max_steps is zero.
TEST_F(MissionControl, StepCountIsZeroWhenMaxStepsIsZero) {
    EXPECT_EQ(makeMission(0)->runMission().steps, 0u);
}

// Errors do not shorten the loop — it still runs max_steps times.
TEST_F(MissionControl, StepCountEqualsMaxStepsEvenWithErrors) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(errorStep()));
    EXPECT_EQ(makeMission(4)->runMission().steps, 4u);
}

// Error vector

// A clean Completed run reports no errors.
TEST_F(MissionControl, ErrorsEmptyOnCleanCompleted) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(completedStep()));
    EXPECT_TRUE(makeMission(10)->runMission().errors.empty());
}

// A clean MaxSteps run reports no errors.
TEST_F(MissionControl, ErrorsEmptyOnCleanMaxSteps) {
    EXPECT_TRUE(makeMission(5)->runMission().errors.empty());
}

// A failing step produces an error entry.
TEST_F(MissionControl, ErrorsNonEmptyWhenStepFails) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(errorStep("bad_motor")));
    EXPECT_FALSE(makeMission(3)->runMission().errors.empty());
}

// The error code for a drone-step failure is DRONE_ERROR.
TEST_F(MissionControl, ErrorRefCodeIsDroneError) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(errorStep("any_error")));
    const auto result = makeMission(1)->runMission();
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "DRONE_ERROR");
}

// The error message is carried verbatim from the step result.
TEST_F(MissionControl, ErrorRefMessageMatchesStepResultMessage) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(errorStep("DRONE_HITS_OBSTACLE")));
    const auto result = makeMission(1)->runMission();
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].message, "DRONE_HITS_OBSTACLE");
}

// Multiple non-consecutive errors are all captured (the loop never breaks on Error).
TEST_F(MissionControl, MultipleErrorsAllCaptured) {
    using namespace ::testing;
    EXPECT_CALL(drone_control_, step())
        .WillOnce(Return(errorStep("err_step1")))
        .WillOnce(Return(continueStep()))
        .WillOnce(Return(continueStep()))
        .WillOnce(Return(errorStep("err_step4")));
    const auto result = makeMission(4)->runMission();
    ASSERT_EQ(result.errors.size(), 2u);
    EXPECT_EQ(result.errors[0].message, "err_step1");
    EXPECT_EQ(result.errors[1].message, "err_step4");
}

// Every failed step contributes an error entry.
TEST_F(MissionControl, ErrorsContainAllFailedStepMessages) {
    using namespace ::testing;
    EXPECT_CALL(drone_control_, step())
        .WillOnce(Return(errorStep("e1")))
        .WillOnce(Return(errorStep("e2")))
        .WillOnce(Return(errorStep("e3")));
    EXPECT_EQ(makeMission(3)->runMission().errors.size(), 3u);
}

// Loop call-count behaviour

// All-Continue runs step() exactly max_steps times.
TEST_F(MissionControl, LoopCallsStepExactlyMaxStepsTimesWhenAllContinue) {
    using namespace ::testing;
    EXPECT_CALL(drone_control_, step()).Times(6);
    (void)makeMission(6)->runMission();
}

// Errors do not break the loop — step() is still called max_steps times.
TEST_F(MissionControl, LoopCallsStepExactlyMaxStepsTimesWhenErrorsOccur) {
    using namespace ::testing;
    EXPECT_CALL(drone_control_, step()).Times(5).WillRepeatedly(Return(errorStep()));
    (void)makeMission(5)->runMission();
}

// Completed breaks the loop immediately.
TEST_F(MissionControl, LoopBreaksImmediatelyOnCompleted) {
    using namespace ::testing;
    EXPECT_CALL(drone_control_, step())
        .Times(2)
        .WillOnce(Return(continueStep()))
        .WillOnce(Return(completedStep()));
    (void)makeMission(10)->runMission();
}

// save() behaviour

// save() is called exactly once on a Completed mission.
TEST_F(MissionControl, SaveCalledOnceOnCompleted) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(completedStep()));
    EXPECT_CALL(output_map_, save(_)).Times(1);
    (void)makeMission(10)->runMission();
}

// save() is called exactly once on a MaxSteps mission.
TEST_F(MissionControl, SaveCalledOnceOnMaxSteps) {
    using namespace ::testing;
    EXPECT_CALL(output_map_, save(_)).Times(1);
    (void)makeMission(3)->runMission();
}

// save() is called exactly once even when the mission errored.
TEST_F(MissionControl, SaveCalledOnceOnError) {
    using namespace ::testing;
    ON_CALL(drone_control_, step()).WillByDefault(Return(errorStep()));
    EXPECT_CALL(output_map_, save(_)).Times(1);
    (void)makeMission(3)->runMission();
}

// save() is called with the exact output path supplied to the constructor.
TEST_F(MissionControl, SaveCalledWithExactOutputPath) {
    using namespace ::testing;
    const std::filesystem::path expected{"/tmp/specific_mission_output.npy"};
    EXPECT_CALL(output_map_, save(expected)).Times(1);
    (void)makeMission(1, expected)->runMission();
}
