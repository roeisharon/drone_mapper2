#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/drone/DroneControlImpl.h>

#include "mocks/MockILidar.h"
#include "mocks/MockIGPS.h"
#include "mocks/MockIDroneMovement.h"
#include "mocks/MockIMutableMap3D.h"
#include "mocks/MockIMappingAlgorithm.h"

#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace {

using drone_mapper::cm;
using drone_mapper::deg;

// Matches a PhysicalLength equal to expected_cm (within a tiny tolerance).
MATCHER_P(CmEq, expected_cm, "physical length == " + std::to_string(expected_cm) + " cm") {
    return std::abs(arg.numerical_value_in(cm) - expected_cm) <= 1e-6;
}

// Matches a HorizontalAngle equal to expected_deg.
MATCHER_P(DegEq, expected_deg, "angle == " + std::to_string(expected_deg) + " deg") {
    return std::abs(arg.numerical_value_in(deg) - expected_deg) <= 1e-6;
}

// Drone config with the per-command movement limits the clamping tests rely on.
drone_mapper::types::DroneConfigData makeConfig(double max_rotate_deg  = 45.0,
                                                double max_advance_cm = 50.0,
                                                double max_elevate_cm = 40.0) {
    drone_mapper::types::DroneConfigData cfg;
    cfg.radius      = 30.0 * cm;
    cfg.max_rotate  = max_rotate_deg * deg;
    cfg.max_advance = max_advance_cm * cm;
    cfg.max_elevate = max_elevate_cm * cm;
    return cfg;
}

// A MapConfig with non-zero resolution so ScanResultToVoxels::applyToMap actually walks the
// beam (it sub-steps at 0.1×resolution and bails if resolution is 0).
drone_mapper::types::MapConfig mapConfig10() {
    using namespace drone_mapper;
    return types::MapConfig{
        types::MappingBounds{0.0 * x_extent[cm], 200.0 * x_extent[cm],
                             0.0 * y_extent[cm], 200.0 * y_extent[cm],
                             0.0 * z_extent[cm], 200.0 * z_extent[cm]},
        Position3D{}, 10.0 * cm};
}

drone_mapper::Orientation forwardOrientation() {
    return {0.0 * drone_mapper::horizontal_angle[drone_mapper::deg],
            0.0 * drone_mapper::altitude_angle[drone_mapper::deg]};
}

// A Working command carrying a movement and (optionally) a forward scan request.
drone_mapper::types::MappingStepCommand workingCmd(drone_mapper::types::MovementCommand mv,
                                                   bool with_scan = true) {
    return {mv,
            with_scan ? std::optional<drone_mapper::Orientation>{forwardOrientation()}
                      : std::optional<drone_mapper::Orientation>{std::nullopt},
            drone_mapper::types::AlgorithmStatus::Working};
}

drone_mapper::types::MappingStepCommand advanceStepCmd(double dist_cm, bool with_scan = true) {
    return workingCmd({drone_mapper::types::MovementCommandType::Advance,
                       drone_mapper::types::RotationDirection::Left, 0.0 * deg, dist_cm * cm},
                      with_scan);
}

drone_mapper::types::MappingStepCommand rotateStepCmd(drone_mapper::types::RotationDirection dir,
                                                      double angle_deg, bool with_scan = true) {
    return workingCmd({drone_mapper::types::MovementCommandType::Rotate,
                       dir, angle_deg * deg, 0.0 * cm}, with_scan);
}

drone_mapper::types::MappingStepCommand elevateStepCmd(double dist_cm, bool with_scan = true) {
    return workingCmd({drone_mapper::types::MovementCommandType::Elevate,
                       drone_mapper::types::RotationDirection::Left, 0.0 * deg, dist_cm * cm},
                      with_scan);
}

drone_mapper::types::MappingStepCommand finishedCmd() {
    return {std::nullopt, std::nullopt, drone_mapper::types::AlgorithmStatus::Finished};
}

drone_mapper::types::MappingStepCommand finishedWithUnmappableCmd() {
    return {std::nullopt, std::nullopt,
            drone_mapper::types::AlgorithmStatus::FinishedWithUnmappableVoxels};
}

drone_mapper::types::MappingStepCommand hoverWorkingCmd() {
    return {drone_mapper::types::MovementCommand{drone_mapper::types::MovementCommandType::Hover},
            forwardOrientation(), drone_mapper::types::AlgorithmStatus::Working};
}

} // namespace

class DroneControl : public ::testing::Test {
protected:
    void SetUp() override {
        using namespace ::testing;
        using drone_mapper::types::VoxelOccupancy;
        using drone_mapper::types::MovementResult;
        using drone_mapper::Position3D;
        using drone_mapper::Orientation;

        ON_CALL(gps_, position()).WillByDefault(Return(Position3D{}));  // origin
        ON_CALL(gps_, heading()).WillByDefault(Return(Orientation{}));  // facing east (0°)

        // Output map: every cell Unmapped (not Occupied), in-bounds, with a real resolution so
        // applyToMap proceeds. isInBounds is mocked true so the converter does not early-return.
        ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Unmapped));
        ON_CALL(output_map_, isInBounds(_)).WillByDefault(Return(true));
        ON_CALL(output_map_, getMapConfig()).WillByDefault(Return(mapConfig10()));

        ON_CALL(lidar_, scan(_)).WillByDefault(Return(drone_mapper::types::LidarScanResult{}));

        ON_CALL(movement_, rotate(_, _)).WillByDefault(Return(MovementResult{true, {}}));
        ON_CALL(movement_, advance(_)).WillByDefault(Return(MovementResult{true, {}}));
        ON_CALL(movement_, elevate(_)).WillByDefault(Return(MovementResult{true, {}}));

        ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(10.0)));

        drone_ = std::make_unique<drone_mapper::DroneControlImpl>(
            makeConfig(), drone_mapper::types::MissionConfigData{},
            lidar_, gps_, movement_, output_map_, *algo_);
    }

    ::testing::NiceMock<drone_mapper::mocks::MockILidar>            lidar_;
    ::testing::NiceMock<drone_mapper::mocks::MockIGPS>             gps_;
    ::testing::NiceMock<drone_mapper::mocks::MockIDroneMovement>   movement_;
    ::testing::NiceMock<drone_mapper::mocks::MockIMutableMap3D>    output_map_;
    ::testing::NiceMock<drone_mapper::mocks::MockIMappingAlgorithm> algo_raw_;
    drone_mapper::mocks::MockIMappingAlgorithm* algo_{&algo_raw_};
    std::unique_ptr<drone_mapper::DroneControlImpl> drone_;
};

// Step status

// A normal working step returns Continue.
TEST_F(DroneControl, StepReturnsContinueOnNormalStep) {
    EXPECT_EQ(drone_->step().status, drone_mapper::types::DroneStepStatus::Continue);
}

// AlgorithmStatus::Finished → DroneStepStatus::Completed.
TEST_F(DroneControl, StepReturnsCompletedWhenAlgorithmFinished) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(finishedCmd()));
    EXPECT_EQ(drone_->step().status, drone_mapper::types::DroneStepStatus::Completed);
}

// FinishedWithUnmappableVoxels also maps to Completed (both finish statuses are terminal-success).
TEST_F(DroneControl, StepReturnsCompletedWhenAlgorithmFinishedWithUnmappable) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(finishedWithUnmappableCmd()));
    EXPECT_EQ(drone_->step().status, drone_mapper::types::DroneStepStatus::Completed);
}

// A failed advance surfaces as a step Error carrying the movement message.
TEST_F(DroneControl, StepReturnsErrorWhenMovementFails) {
    using namespace ::testing;
    ON_CALL(movement_, advance(_))
        .WillByDefault(Return(drone_mapper::types::MovementResult{false, "motor_stall"}));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, drone_mapper::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "motor_stall");
}

// A failed rotate surfaces as a step Error.
TEST_F(DroneControl, StepReturnsErrorWhenRotateFails) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(rotateStepCmd(drone_mapper::types::RotationDirection::Left, 10.0)));
    ON_CALL(movement_, rotate(_, _))
        .WillByDefault(Return(drone_mapper::types::MovementResult{false, "servo_error"}));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, drone_mapper::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "servo_error");
}

// A failed elevate surfaces as a step Error.
TEST_F(DroneControl, StepReturnsErrorWhenElevateFails) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(10.0)));
    ON_CALL(movement_, elevate(_))
        .WillByDefault(Return(drone_mapper::types::MovementResult{false, "prop_fault"}));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, drone_mapper::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "prop_fault");
}

// A collision refusal (DRONE_HITS_OBSTACLE) is routine exploration, not a fatal fault: the step
// continues so the planner can reroute. This is what distinguishes it from a genuine movement fault
// (motor_stall etc.), which still errors above.
TEST_F(DroneControl, StepContinuesWhenMovementRefusedByCollision) {
    using namespace ::testing;
    ON_CALL(movement_, advance(_))
        .WillByDefault(Return(drone_mapper::types::MovementResult{false, "DRONE_HITS_OBSTACLE"}));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, drone_mapper::types::DroneStepStatus::Continue);
}

// Obstacle detection

// Current cell Occupied → DRONE_HITS_OBSTACLE error.
TEST_F(DroneControl, StepReturnsObstacleErrorWhenCurrentPositionIsOccupied) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(drone_mapper::types::VoxelOccupancy::Occupied));
    const auto result = drone_->step();
    EXPECT_EQ(result.status, drone_mapper::types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "DRONE_HITS_OBSTACLE");
}

// The obstacle early-return does not advance the step index.
TEST_F(DroneControl, ObstacleErrorDoesNotIncrementStepIndex) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(drone_mapper::types::VoxelOccupancy::Occupied));
    EXPECT_EQ(drone_->state().step_index, 0u);
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 0u);
}

// The obstacle early-return happens before any lidar/algorithm call.
TEST_F(DroneControl, ObstacleErrorDoesNotCallLidarOrAlgorithm) {
    using namespace ::testing;
    ON_CALL(output_map_, atVoxel(_)).WillByDefault(Return(drone_mapper::types::VoxelOccupancy::Occupied));
    EXPECT_CALL(lidar_, scan(_)).Times(0);
    EXPECT_CALL(*algo_, nextStep(_, _)).Times(0);
    (void)drone_->step();
}

// Algorithm / lidar interaction

// The algorithm is consulted exactly once per step.
TEST_F(DroneControl, StepCallsAlgorithmBeforeLidar) {
    using namespace ::testing;
    EXPECT_CALL(*algo_, nextStep(_, _)).Times(1).WillOnce(Return(advanceStepCmd(10.0)));
    (void)drone_->step();
}

// No scan_orientation in the command → lidar is not called.
TEST_F(DroneControl, StepCallsLidarOnlyWhenScanOrientationRequested) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(10.0, /*with_scan=*/false)));
    EXPECT_CALL(lidar_, scan(_)).Times(0);
    (void)drone_->step();
}

// The lidar is scanned with the orientation the algorithm requested.
TEST_F(DroneControl, StepCallsLidarScanWithAlgorithmOrientation) {
    using namespace ::testing;
    EXPECT_CALL(lidar_, scan(Truly([](const drone_mapper::Orientation& o) {
        return o.horizontal.numerical_value_in(deg) == 0.0;
    }))).Times(1);
    (void)drone_->step();
}

// A lidar hit is written into the output map as Occupied (via ScanResultToVoxels::applyToMap).
TEST_F(DroneControl, StepWritesOccupiedToOutputMapWhenLidarHits) {
    using namespace ::testing;
    drone_mapper::types::LidarHit hit;
    hit.distance = 5.0 * cm;
    hit.angle    = drone_mapper::Orientation{};
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(drone_mapper::types::LidarScanResult{hit}));
    // Permit the Empty path-clearing writes; require at least one Occupied (the hit endpoint).
    // The catch-all is declared first so the specific Occupied expectation is matched first.
    EXPECT_CALL(output_map_, set(_, _)).Times(AnyNumber());
    EXPECT_CALL(output_map_, set(_, drone_mapper::types::VoxelOccupancy::Occupied)).Times(AtLeast(1));
    (void)drone_->step();
}

// An empty scan writes no Occupied voxels — only the drone's own cell is recorded Empty.
TEST_F(DroneControl, StepWithEmptyScanWritesNoOccupied) {
    using namespace ::testing;
    using drone_mapper::types::VoxelOccupancy;
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(drone_mapper::types::LidarScanResult{}));
    EXPECT_CALL(output_map_, set(_, VoxelOccupancy::Empty)).Times(AtLeast(1));
    EXPECT_CALL(output_map_, set(_, VoxelOccupancy::Occupied)).Times(0);
    (void)drone_->step();
}

// The drone records its own (provably free) cell Empty, filling open space the lidar misses.
TEST_F(DroneControl, StepMarksCurrentCellEmptyInOutputMap) {
    using namespace ::testing;
    using drone_mapper::types::VoxelOccupancy;
    using drone_mapper::Position3D;
    const Position3D here{1.0 * drone_mapper::x_extent[cm], 2.0 * drone_mapper::y_extent[cm],
                          3.0 * drone_mapper::z_extent[cm]};
    ON_CALL(gps_, position()).WillByDefault(Return(here));
    EXPECT_CALL(output_map_, set(Truly([](const Position3D& p) {
                                     return p.x.numerical_value_in(cm) == 1.0 &&
                                            p.y.numerical_value_in(cm) == 2.0 &&
                                            p.z.numerical_value_in(cm) == 3.0;
                                 }),
                                 VoxelOccupancy::Empty)).Times(AtLeast(1));
    (void)drone_->step();
}

// The lidar's own config() (ILidar::config()) reaches applyToMap: with a large z_max a miss empties
// a long ray (many Empty writes), which would not happen with the default zero-config. Confirms
// step() reads the config from the injected lidar rather than a stale/zero default.
TEST_F(DroneControl, LidarConfigFromLidarIsUsedByScanConversion) {
    using namespace ::testing;
    using drone_mapper::types::VoxelOccupancy;
    ON_CALL(lidar_, config())
        .WillByDefault(Return(drone_mapper::types::LidarConfigData{0.0 * cm, 50.0 * cm, 1.0 * cm, 1}));
    // A miss: max-double distance → applyToMap empties the ray out to z_max (50 cm at 1 cm steps).
    drone_mapper::types::LidarHit miss;
    miss.distance = std::numeric_limits<double>::max() * cm;
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(drone_mapper::types::LidarScanResult{miss}));
    EXPECT_CALL(output_map_, set(_, VoxelOccupancy::Empty)).Times(AtLeast(10));
    (void)drone_->step();
}

// latest_scan threading

// The first step passes nullptr as latest_scan (no prior scan exists).
TEST_F(DroneControl, FirstCallPassesNullScanToAlgorithm) {
    using namespace ::testing;
    const drone_mapper::types::LidarScanResult* captured = reinterpret_cast<const drone_mapper::types::LidarScanResult*>(0x1);
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault([&](const drone_mapper::types::DroneState&,
                           const drone_mapper::types::LidarScanResult* scan) {
            captured = scan;
            return finishedCmd();
        });
    (void)drone_->step();
    EXPECT_EQ(captured, nullptr);
}

// The second step passes the scan captured on the first step (non-null).
TEST_F(DroneControl, SecondCallPassesScanResultFromFirstScan) {
    using namespace ::testing;
    drone_mapper::types::LidarHit hit;
    hit.distance = 5.0 * cm;
    ON_CALL(lidar_, scan(_)).WillByDefault(Return(drone_mapper::types::LidarScanResult{hit}));
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(10.0)));
    (void)drone_->step(); // first step scans → latest_scan stored

    const drone_mapper::types::LidarScanResult* second = nullptr;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce([&](const drone_mapper::types::DroneState&,
                      const drone_mapper::types::LidarScanResult* p) {
            second = p;
            return finishedCmd();
        });
    (void)drone_->step();
    EXPECT_NE(second, nullptr);
}

// Clamping

TEST_F(DroneControl, AdvanceNotClampedWhenBelowMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(30.0)));
    EXPECT_CALL(movement_, advance(CmEq(30.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, AdvanceClampedWhenAboveMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(200.0)));
    EXPECT_CALL(movement_, advance(CmEq(50.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, AdvanceAtExactMaxNotClamped) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(50.0)));
    EXPECT_CALL(movement_, advance(CmEq(50.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, RotateNotClampedWhenBelowMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(rotateStepCmd(drone_mapper::types::RotationDirection::Left, 20.0)));
    EXPECT_CALL(movement_, rotate(drone_mapper::types::RotationDirection::Left, DegEq(20.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, RotateClampedWhenAboveMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _))
        .WillByDefault(Return(rotateStepCmd(drone_mapper::types::RotationDirection::Right, 90.0)));
    EXPECT_CALL(movement_, rotate(drone_mapper::types::RotationDirection::Right, DegEq(45.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, ElevateNotClampedWhenBelowMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(20.0)));
    EXPECT_CALL(movement_, elevate(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, ElevateClampedWhenAboveMax) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(80.0)));
    EXPECT_CALL(movement_, elevate(CmEq(40.0))).Times(1);
    (void)drone_->step();
}

// A sub-cell request is floored to whole cells (never rounded up), so it never exceeds the requested
// magnitude, and the descent sign is preserved: -15cm at 10cm resolution → 1 cell = -10cm (not -20).
TEST_F(DroneControl, ElevateNegativeSignPreserved) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(-15.0)));
    EXPECT_CALL(movement_, elevate(CmEq(-10.0))).Times(1);
    (void)drone_->step();
}

// floor (not round): a request between cell multiples is truncated DOWN so the executed move never
// exceeds the requested/validated distance. 25cm at 10cm resolution → 2 cells = 20cm (round would give 30).
TEST_F(DroneControl, AdvanceFlooredToWholeCellsNeverExceedsRequest) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(25.0)));
    EXPECT_CALL(movement_, advance(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

TEST_F(DroneControl, ElevateNegativeClampedSignPreserved) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(-80.0)));
    EXPECT_CALL(movement_, elevate(CmEq(-40.0))).Times(1);
    (void)drone_->step();
}

// Checkpoint A — translations are clamped to WHOLE cells so they land on cell centres even when the
// per-command limit is not a multiple of the resolution. With max_advance 25cm and res 10cm, the
// largest whole-cell move is 2 cells = 20cm (the old continuous clamp would have allowed sub-cell 25).
TEST_F(DroneControl, AdvanceClampedToWholeCellsWhenMaxNotResolutionMultiple) {
    using namespace ::testing;
    drone_ = std::make_unique<drone_mapper::DroneControlImpl>(
        makeConfig(45.0, 25.0), drone_mapper::types::MissionConfigData{},
        lidar_, gps_, movement_, output_map_, *algo_);
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(advanceStepCmd(200.0)));
    EXPECT_CALL(movement_, advance(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

// Same for elevate: max_elevate 25cm, res 10cm → 2 whole cells = 20cm.
TEST_F(DroneControl, ElevateClampedToWholeCellsWhenMaxNotResolutionMultiple) {
    using namespace ::testing;
    drone_ = std::make_unique<drone_mapper::DroneControlImpl>(
        makeConfig(45.0, 50.0, 25.0), drone_mapper::types::MissionConfigData{},
        lidar_, gps_, movement_, output_map_, *algo_);
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(elevateStepCmd(200.0)));
    EXPECT_CALL(movement_, elevate(CmEq(20.0))).Times(1);
    (void)drone_->step();
}

// No-movement commands

// A Hover command does not invoke any movement API and still returns Continue.
TEST_F(DroneControl, HoverMovementDoesNotCallMovementAPI) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(hoverWorkingCmd()));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    EXPECT_EQ(drone_->step().status, drone_mapper::types::DroneStepStatus::Continue);
}

// A command with no movement field does not invoke any movement API.
TEST_F(DroneControl, CommandWithNoMovementDoesNotCallMovementAPI) {
    using namespace ::testing;
    const drone_mapper::types::MappingStepCommand no_move{
        std::nullopt, forwardOrientation(), drone_mapper::types::AlgorithmStatus::Working};
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(no_move));
    EXPECT_CALL(movement_, advance(_)).Times(0);
    EXPECT_CALL(movement_, rotate(_, _)).Times(0);
    EXPECT_CALL(movement_, elevate(_)).Times(0);
    EXPECT_EQ(drone_->step().status, drone_mapper::types::DroneStepStatus::Continue);
}

// Step index tracking

TEST_F(DroneControl, StepIndexStartsAtZero) {
    EXPECT_EQ(drone_->state().step_index, 0u);
}

TEST_F(DroneControl, StepIndexIncrementsAfterContinue) {
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 1u);
}

TEST_F(DroneControl, StepIndexIncrementsAfterCompleted) {
    using namespace ::testing;
    ON_CALL(*algo_, nextStep(_, _)).WillByDefault(Return(finishedCmd()));
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 1u);
}

TEST_F(DroneControl, StepIndexIncrementsEvenOnMovementFailure) {
    using namespace ::testing;
    ON_CALL(movement_, advance(_))
        .WillByDefault(Return(drone_mapper::types::MovementResult{false, "err"}));
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 1u);
}

TEST_F(DroneControl, StepIndexAccumulatesAcrossMultipleCalls) {
    (void)drone_->step();
    (void)drone_->step();
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 3u);
}

// state()

TEST_F(DroneControl, StateReflectsCurrentGpsPosition) {
    using namespace ::testing;
    const drone_mapper::Position3D expected{7.0 * drone_mapper::x_extent[cm],
                                            3.0 * drone_mapper::y_extent[cm],
                                            1.0 * drone_mapper::z_extent[cm]};
    ON_CALL(gps_, position()).WillByDefault(Return(expected));
    const auto s = drone_->state();
    EXPECT_DOUBLE_EQ(s.position.x.numerical_value_in(cm), 7.0);
    EXPECT_DOUBLE_EQ(s.position.y.numerical_value_in(cm), 3.0);
    EXPECT_DOUBLE_EQ(s.position.z.numerical_value_in(cm), 1.0);
}

TEST_F(DroneControl, StateReflectsCurrentGpsHeading) {
    using namespace ::testing;
    const drone_mapper::Orientation expected{30.0 * deg, {}};
    ON_CALL(gps_, heading()).WillByDefault(Return(expected));
    EXPECT_DOUBLE_EQ(drone_->state().heading.horizontal.numerical_value_in(deg), 30.0);
}

TEST_F(DroneControl, StateStepIndexMatchesCallCount) {
    (void)drone_->step();
    (void)drone_->step();
    EXPECT_EQ(drone_->state().step_index, 2u);
}

// Step index / state passed to the algorithm

TEST_F(DroneControl, NextStepReceivesStepIndexZeroOnFirstCall) {
    using namespace ::testing;
    EXPECT_CALL(*algo_, nextStep(Field(&drone_mapper::types::DroneState::step_index, 0u), _))
        .Times(1).WillOnce(Return(finishedCmd()));
    (void)drone_->step();
}

TEST_F(DroneControl, NextStepReceivesIncrementedStepIndexOnLaterCalls) {
    using namespace ::testing;
    drone_mapper::types::DroneState third_call_state;
    EXPECT_CALL(*algo_, nextStep(_, _))
        .WillOnce(Return(advanceStepCmd(10.0)))
        .WillOnce(Return(advanceStepCmd(10.0)))
        .WillOnce(DoAll(SaveArg<0>(&third_call_state), Return(finishedCmd())));
    (void)drone_->step();
    (void)drone_->step();
    (void)drone_->step();
    EXPECT_EQ(third_call_state.step_index, 2u);
}

TEST_F(DroneControl, NextStepReceivesCurrentPosition) {
    using namespace ::testing;
    const drone_mapper::Position3D test_pos{5.0 * drone_mapper::x_extent[cm],
                                            3.0 * drone_mapper::y_extent[cm],
                                            2.0 * drone_mapper::z_extent[cm]};
    ON_CALL(gps_, position()).WillByDefault(Return(test_pos));
    EXPECT_CALL(*algo_, nextStep(Truly([](const drone_mapper::types::DroneState& s) {
                                     return std::abs(s.position.x.numerical_value_in(cm) - 5.0) < 1e-9 &&
                                            std::abs(s.position.y.numerical_value_in(cm) - 3.0) < 1e-9 &&
                                            std::abs(s.position.z.numerical_value_in(cm) - 2.0) < 1e-9;
                                 }),
                                 _)).Times(1).WillOnce(Return(finishedCmd()));
    (void)drone_->step();
}
