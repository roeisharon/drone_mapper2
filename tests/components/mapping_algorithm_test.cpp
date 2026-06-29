#include <gtest/gtest.h>

#include <drone_mapper/drone/MappingAlgorithmImpl.h>
#include <drone_mapper/map/IMap3D.h>
#include <drone_mapper/Units.h>

#include <cmath>
#include <map>
#include <numbers>
#include <set>
#include <tuple>

// A simple IMap3D backed by a std::map of (ix, iy, iz) → VoxelOccupancy, so tests can inject
// occupancy without a real NpyArray. atVoxel() quantises the query position to the map's
// resolution before lookup, matching the grid-cell keys the algorithm produces.
namespace {

class TestMap final : public drone_mapper::IMap3D {
public:
    explicit TestMap(double res_cm)
        : config_{{}, drone_mapper::Position3D{}, res_cm * drone_mapper::cm} {}

    [[nodiscard]] drone_mapper::types::VoxelOccupancy
    atVoxel(const drone_mapper::Position3D& pos) const override {
        const auto key = posToKey(pos);
        // Optional cubic grid-cell bounds: cells outside report OutOfBounds, exactly as the
        // real Map3DImpl does beyond the loaded array's extent.
        if (has_bounds_) {
            const auto [kx, ky, kz] = key;
            if (kx < min_cell_ || kx > max_cell_ ||
                ky < min_cell_ || ky > max_cell_ ||
                kz < min_cell_ || kz > max_cell_) {
                return drone_mapper::types::VoxelOccupancy::OutOfBounds;
            }
        }
        const auto it = data_.find(key);
        if (it == data_.end()) return drone_mapper::types::VoxelOccupancy::Unmapped;
        return it->second;
    }

    [[nodiscard]] drone_mapper::types::MapConfig getMapConfig() const override { return config_; }

    [[nodiscard]] bool isInBounds(const drone_mapper::Position3D& pos) const override {
        return atVoxel(pos) != drone_mapper::types::VoxelOccupancy::OutOfBounds;
    }

    // Writes a voxel — used by test setup.
    void setVoxel(const drone_mapper::Position3D& pos, drone_mapper::types::VoxelOccupancy occ) {
        data_[posToKey(pos)] = occ;
    }

    // Restricts the in-bounds region to the inclusive cubic grid range [min_cell, max_cell].
    void setGridBounds(int min_cell, int max_cell) {
        has_bounds_ = true;
        min_cell_   = min_cell;
        max_cell_   = max_cell;
    }

private:
    using Key = std::tuple<int, int, int>;

    Key posToKey(const drone_mapper::Position3D& p) const {
        // Match Map3DImpl exactly: world -> index is floor(world / res), NOT round. The algorithm
        // samples cell centres at (cell + 0.5) * res, so floor maps that back to the cell index;
        // round would shift it by one and the injected occupancy would never line up.
        const double step = config_.resolution.numerical_value_in(drone_mapper::cm);
        return {
            static_cast<int>(std::floor(p.x.numerical_value_in(drone_mapper::cm) / step)),
            static_cast<int>(std::floor(p.y.numerical_value_in(drone_mapper::cm) / step)),
            static_cast<int>(std::floor(p.z.numerical_value_in(drone_mapper::cm) / step)),
        };
    }

    std::map<Key, drone_mapper::types::VoxelOccupancy> data_;
    drone_mapper::types::MapConfig config_;
    bool has_bounds_ = false;
    int  min_cell_   = 0;
    int  max_cell_   = 0;
};

// Builds a DroneState at the given world position (cm) with an optional heading (°).
drone_mapper::types::DroneState stateAt(double x, double y, double z, double heading_deg = 0.0) {
    using namespace drone_mapper;
    return {
        Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]},
        Orientation{heading_deg * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        0,
    };
}

// Wraps a Position3D for use with TestMap::setVoxel().
drone_mapper::Position3D posAt(double x, double y, double z) {
    using namespace drone_mapper;
    return {x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

// Result of driving the algorithm to completion in a kinematic simulation.
struct SimResult {
    bool finished         = false;
    bool entered_obstacle = false; // did the drone ever occupy an Occupied cell?
    int  iterations       = 0;
};

// Runs the algorithm with a simple kinematic model: each returned movement command is applied to
// the drone pose (the algorithm returns full, unclamped distances), the new state is fed back, and
// we continue until Finished or the cap. Tracks whether the drone ever stood in an Occupied cell.
SimResult simulate(drone_mapper::MappingAlgorithmImpl& algo, const TestMap& map,
                   double x0, double y0, double z0, int cap = 3000) {
    using namespace drone_mapper;
    using T = types::MovementCommandType;
    double x = x0, y = y0, z = z0, heading = 0.0;
    SimResult r;
    for (; r.iterations < cap; ++r.iterations) {
        if (map.atVoxel(posAt(x, y, z)) == types::VoxelOccupancy::Occupied) {
            r.entered_obstacle = true;
        }
        const auto cmd = algo.nextStep(stateAt(x, y, z, heading), nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished) {
            r.finished = true;
            break;
        }
        if (cmd.movement) {
            const auto& mv = *cmd.movement;
            if (mv.type == T::Rotate) {
                const double a = mv.angle.numerical_value_in(deg);
                heading += (mv.rotation == types::RotationDirection::Left) ? a : -a;
                while (heading >= 360.0) heading -= 360.0;
                while (heading <    0.0) heading += 360.0;
            } else if (mv.type == T::Advance) {
                const double d   = mv.distance.numerical_value_in(cm);
                const double rad = heading * std::numbers::pi / 180.0;
                x += d * std::cos(rad);
                y += d * std::sin(rad);
            } else if (mv.type == T::Elevate) {
                z += mv.distance.numerical_value_in(cm);
            }
        }
    }
    return r;
}

} // namespace

// Fixture named "MappingAlgorithm" so --gtest_filter=MappingAlgorithm.* works. map_ and the
// config members must be declared before algo_ (algo_ binds references to them).
class MappingAlgorithm : public ::testing::Test {
protected:
    TestMap map_{10.0}; // 10 cm grid step
    drone_mapper::types::MissionConfigData mission_config_{};
    drone_mapper::types::LidarConfigData   lidar_config_{};
    drone_mapper::types::DroneConfigData   drone_config_{};
    drone_mapper::MappingAlgorithmImpl     algo_{mission_config_, lidar_config_, drone_config_, map_};

    // Marks every cell in the inclusive cubic grid range [lo,hi] Empty in map_ (confirmed free).
    // Scan-before-enter only moves into confirmed-Empty cells, and the kinematic simulate() does not
    // model the lidar revealing cells, so the explorable room must be pre-confirmed for these tests.
    void fillEmpty(int lo, int hi) {
        using V = drone_mapper::types::VoxelOccupancy;
        for (int x = lo; x <= hi; ++x)
            for (int y = lo; y <= hi; ++y)
                for (int z = lo; z <= hi; ++z)
                    map_.setVoxel(posAt(x * 10.0, y * 10.0, z * 10.0), V::Empty);
    }
};

// nextStep: basic navigation commands

// On the first call (0,0,0) is visited and 6 neighbours are enqueued → not Finished.
TEST_F(MappingAlgorithm, FirstCallReturnsNonFinishedWhenFrontierNonEmpty) {
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_NE(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
}

// A confirmed-Empty neighbour gives somewhere to navigate → a (non-Hover) movement command.
TEST_F(MappingAlgorithm, FirstCallHasMovementCommand) {
    map_.setVoxel(posAt(10, 0, 0), drone_mapper::types::VoxelOccupancy::Empty); // confirmed → advance
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_TRUE(cmd.movement.has_value());
    EXPECT_NE(cmd.movement->type, drone_mapper::types::MovementCommandType::Hover);
}

// All 6 neighbours occupied → after visiting (0,0,0) the frontier is empty → Finished.
TEST_F(MappingAlgorithm, ReturnsFinishedWhenAllNeighboursOccupied) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
}

// Regression: OutOfBounds cells must not be navigable, else the frontier leaks outside the map
// and the algorithm never finishes. Box the drone into the single in-bounds cell → Finished.
TEST_F(MappingAlgorithm, FinishedWhenAllNeighboursOutOfBounds) {
    map_.setGridBounds(0, 0);
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
}

// A small bounded empty room (3×3×3), pre-confirmed Empty, is fully explored and Finished in budget.
TEST_F(MappingAlgorithm, TerminatesInBoundedEmptyRoom) {
    map_.setGridBounds(0, 2);
    fillEmpty(0, 2);
    const auto r = simulate(algo_, map_, 0.0, 0.0, 0.0);
    EXPECT_TRUE(r.finished) << "did not finish (looped " << r.iterations << " times)";
}

// 3×3×3 room with the centre occupied: explore the 26 open cells, never enter the obstacle, finish.
TEST_F(MappingAlgorithm, NeverNavigatesIntoOccupiedCellAndFinishes) {
    map_.setGridBounds(0, 2);
    fillEmpty(0, 2);
    map_.setVoxel(posAt(10, 10, 10), drone_mapper::types::VoxelOccupancy::Occupied); // grid (1,1,1)
    const auto r = simulate(algo_, map_, 0.0, 0.0, 0.0);
    EXPECT_FALSE(r.entered_obstacle) << "drone navigated into the occupied cell";
    EXPECT_TRUE(r.finished) << "did not finish (looped " << r.iterations << " times)";
}

// Only +X reachable (confirmed Empty) and the drone faces 0° → Advance, positive distance.
// The drone sits at the centre of cell (0,0,0) = (5,5,5): the algorithm plans cell-centre to
// cell-centre, so a corner start would skew the heading by half a voxel.
TEST_F(MappingAlgorithm, AdvanceCommandIssuedWhenFacingTarget) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Empty);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5, 0.0), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Advance);
    EXPECT_GT(cmd.movement->distance.numerical_value_in(drone_mapper::cm), 0.0);
}

// Advance distance equals exactly one grid step (10 cm) from the cell centre toward the next.
TEST_F(MappingAlgorithm, AdvanceDistanceEqualsOneGridStep) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Empty);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5, 0.0), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Advance);
    EXPECT_NEAR(cmd.movement->distance.numerical_value_in(drone_mapper::cm), 10.0, 0.1);
}

// Only +X reachable but the drone faces 180° (opposite) → must Rotate first.
TEST_F(MappingAlgorithm, RotateCommandIssuedWhenNotFacingTarget) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0, 180.0), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Rotate);
    EXPECT_GT(cmd.movement->angle.numerical_value_in(drone_mapper::deg), 0.0);
}

// Target in +Y (desired 90°), drone at 0° → diff +90° → rotate Left.
TEST_F(MappingAlgorithm, RotateDirectionLeftForPositiveDiff) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0, 0.0), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Rotate);
    EXPECT_EQ(cmd.movement->rotation, drone_mapper::types::RotationDirection::Left);
}

// Target in -Y (desired 270°), drone at 0° → diff -90° → rotate Right.
TEST_F(MappingAlgorithm, RotateDirectionRightForNegativeDiff) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5, 0.0), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Rotate);
    EXPECT_EQ(cmd.movement->rotation, drone_mapper::types::RotationDirection::Right);
}

// All horizontal neighbours blocked, only ±Z left (+Z confirmed Empty) → Elevate.
TEST_F(MappingAlgorithm, ElevateCommandIssuedForVerticalOnlyTarget) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(0, 0, 10), V::Empty);
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Elevate);
    EXPECT_NE(cmd.movement->distance.numerical_value_in(drone_mapper::cm), 0.0);
}

// Only +Z reachable (confirmed Empty) → Elevate distance must be positive (upward).
TEST_F(MappingAlgorithm, ElevateUpwardHasPositiveDistance) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(0, 0, 10), V::Empty);
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Elevate);
    EXPECT_GT(cmd.movement->distance.numerical_value_in(drone_mapper::cm), 0.0);
}

// Scan orientation

// A working step always requests a scan.
TEST_F(MappingAlgorithm, AlwaysRequestsAScan) {
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_TRUE(cmd.scan_orientation.has_value());
}

// The scan orientation is {0°, 0°} (forward relative to current heading).
TEST_F(MappingAlgorithm, ScanOrientationIsForward) {
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    ASSERT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_DOUBLE_EQ(cmd.scan_orientation->horizontal.numerical_value_in(drone_mapper::deg), 0.0);
}

// When Finished, no movement is commanded. (A final sweep scan may still be requested — the
// algorithm always emits a scan orientation; DroneControl maps Finished → Completed regardless.)
TEST_F(MappingAlgorithm, NoMovementWhenFinished) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
    EXPECT_FALSE(cmd.movement.has_value());
}

// Occupancy blocking

// An Occupied neighbour is never chosen as a target: with +X occupied and ±Y blocked, only ±Z
// remains → Elevate.
TEST_F(MappingAlgorithm, OccupiedVoxelNeverSelectedAsTarget) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(0, 0, 10), V::Empty); // the one open neighbour, confirmed → Elevate
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Elevate);
}

// A PotentiallyOccupied neighbour is treated as an obstacle too (collision safety): +X marked
// PotentiallyOccupied and ±Y/-X occupied → only ±Z remains → Elevate.
TEST_F(MappingAlgorithm, PotentiallyOccupiedNeverSelectedAsTarget) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(0, 0, 10), V::Empty); // the one open neighbour, confirmed → Elevate
    map_.setVoxel(posAt( 10, 0, 0), V::PotentiallyOccupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Elevate);
}

// A cell occupied AFTER being enqueued must be skipped when popped: leave +X and +Z open on visit
// 1, mark +X occupied between calls, then only +Z remains → Elevate.
TEST_F(MappingAlgorithm, OccupiedCellMarkedAfterEnqueueIsSkippedAtPop) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(0, 0, 10), V::Empty); // +Z confirmed open (the surviving target)
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    static_cast<void>(algo_.nextStep(stateAt(5, 5, 5), nullptr)); // visit (0,0,0); +X and +Z queued
    map_.setVoxel(posAt(10, 0, 0), V::Occupied);                  // +X now blocked
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Elevate);
}

// State machine: reactive multi-step navigation

// Only +X reachable: step 1 at heading 180° → Rotate; step 2 at heading 0° → Advance to same target.
TEST_F(MappingAlgorithm, RotateFollowedByAdvanceToSameTarget) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Empty);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd1 = algo_.nextStep(stateAt(5, 5, 5, 180.0), nullptr);
    ASSERT_TRUE(cmd1.movement.has_value());
    EXPECT_EQ(cmd1.movement->type, drone_mapper::types::MovementCommandType::Rotate);
    const auto cmd2 = algo_.nextStep(stateAt(5, 5, 5, 0.0), nullptr);
    ASSERT_TRUE(cmd2.movement.has_value());
    EXPECT_EQ(cmd2.movement->type, drone_mapper::types::MovementCommandType::Advance);
}

// After arriving at a target the algorithm clears it (no infinite Advance to a reached cell).
TEST_F(MappingAlgorithm, ArrivalAtTargetClearsTargetAndPicksNext) {
    static_cast<void>(algo_.nextStep(stateAt(0, 0, 0, 0.0), nullptr)); // set a target
    const auto cmd = algo_.nextStep(stateAt(10, 0, 0, 0.0), nullptr);  // arrived at (10,0,0)
    (void)cmd; // either further navigation or Finished is valid; must not loop forever
    SUCCEED();
}

// The visited_ guard prevents re-expanding the same cell: repeated calls from (0,0,0) do not
// prematurely exhaust the (large) frontier.
TEST_F(MappingAlgorithm, NoDuplicateFrontierEntries) {
    static_cast<void>(algo_.nextStep(stateAt(0, 0, 0), nullptr));
    static_cast<void>(algo_.nextStep(stateAt(0, 0, 0), nullptr));
    static_cast<void>(algo_.nextStep(stateAt(0, 0, 0), nullptr));
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_NE(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
}

// The grid step is derived from the output map resolution: a 50 cm map issues a 50 cm Advance.
TEST_F(MappingAlgorithm, StepCmDerivedFromOutputMapResolution) {
    TestMap map50{50.0};
    drone_mapper::types::MissionConfigData mcfg{};
    drone_mapper::types::LidarConfigData   lcfg{};
    drone_mapper::types::DroneConfigData   dcfg{};
    drone_mapper::MappingAlgorithmImpl     algo50{mcfg, lcfg, dcfg, map50};

    using V = drone_mapper::types::VoxelOccupancy;
    map50.setVoxel(posAt( 50, 0, 0), V::Empty);
    map50.setVoxel(posAt(-50, 0, 0), V::Occupied);
    map50.setVoxel(posAt(0,  50, 0), V::Occupied);
    map50.setVoxel(posAt(0, -50, 0), V::Occupied);
    map50.setVoxel(posAt(0, 0,  50), V::Occupied);
    map50.setVoxel(posAt(0, 0, -50), V::Occupied);
    const auto cmd = algo50.nextStep(stateAt(25, 25, 25, 0.0), nullptr); // centre of cell (0,0,0)
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Advance);
    EXPECT_NEAR(cmd.movement->distance.numerical_value_in(drone_mapper::cm), 50.0, 0.5);
}

// Visiting a second cell expands its neighbours → frontier stays non-empty.
TEST_F(MappingAlgorithm, FrontierExpandsWhenDroneMoves) {
    static_cast<void>(algo_.nextStep(stateAt(0, 0, 0, 0.0), nullptr)); // visit (0,0,0)
    const auto cmd = algo_.nextStep(stateAt(10, 0, 0, 0.0), nullptr);  // visit (10,0,0)
    EXPECT_NE(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
}

// Status is Working while the frontier is non-empty.
TEST_F(MappingAlgorithm, StatusIsWorkingWhenFrontierNonEmpty) {
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Working);
}

// A non-null latest_scan pointer is accepted (map remains the source of truth).
TEST_F(MappingAlgorithm, LatestScanPointerIsAccepted) {
    drone_mapper::types::LidarScanResult dummy_scan;
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), &dummy_scan);
    EXPECT_NE(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
}

// The first call always receives nullptr and must work.
TEST_F(MappingAlgorithm, NullScanPointerOnFirstCallIsValid) {
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Working);
}

// Optimistic exploration: scan while moving

// Facing an Unmapped neighbour, the drone moves toward it (treating Unmapped as passable) while
// also requesting a scan in the same step — exploration is optimistic, not scan-then-enter.
// Only +X is open (Unmapped) and the drone faces it → Advance + scan.
TEST_F(MappingAlgorithm, MovesIntoUnmappedHorizontalCellWhileScanning) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5, 0.0), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Advance);
    EXPECT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Working);
}

// For a vertical-only Unmapped target below, the drone elevates downward and pitches its scan down
// (-45°) toward the cell it is descending into. Only -Z is open.
TEST_F(MappingAlgorithm, ElevatesDownIntoUnmappedCellBelowWithDownwardScan) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Elevate);
    EXPECT_LT(cmd.movement->distance.numerical_value_in(drone_mapper::cm), 0.0) << "descends";
    ASSERT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_DOUBLE_EQ(cmd.scan_orientation->altitude.numerical_value_in(drone_mapper::deg), -45.0);
}

// A confirmed-Empty forward cell is advanced into (the common case once a cell has been scanned).
TEST_F(MappingAlgorithm, AdvancesIntoConfirmedEmptyForwardCell) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt( 10, 0, 0), V::Empty);
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    const auto cmd = algo_.nextStep(stateAt(5, 5, 5, 0.0), nullptr);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, drone_mapper::types::MovementCommandType::Advance);
}

// If a scan reveals the next cell Occupied, the drone never advances into it.
TEST_F(MappingAlgorithm, DoesNotEnterCellRevealedOccupied) {
    using V = drone_mapper::types::VoxelOccupancy;
    map_.setVoxel(posAt(-10, 0, 0), V::Occupied);
    map_.setVoxel(posAt(0,  10, 0), V::Occupied);
    map_.setVoxel(posAt(0, -10, 0), V::Occupied);
    map_.setVoxel(posAt(0, 0,  10), V::Occupied);
    map_.setVoxel(posAt(0, 0, -10), V::Occupied);
    static_cast<void>(algo_.nextStep(stateAt(0, 0, 0, 0.0), nullptr)); // +X Unmapped → scan
    map_.setVoxel(posAt(10, 0, 0), V::Occupied);                       // scan revealed it a wall
    const auto cmd = algo_.nextStep(stateAt(0, 0, 0, 0.0), nullptr);
    // No open neighbour remains → Finished (and certainly never an Advance into the wall).
    if (cmd.movement.has_value()) {
        EXPECT_NE(cmd.movement->type, drone_mapper::types::MovementCommandType::Advance);
    } else {
        EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
    }
}

// Drone footprint (radius) awareness ------------------------------------------------------------

namespace {

using drone_mapper::types::DroneConfigData;
using drone_mapper::types::LidarConfigData;
using drone_mapper::types::MissionConfigData;

// A DroneConfigData whose only meaningful field for these tests is the sphere radius (cm).
DroneConfigData droneWithRadius(double radius_cm) {
    using namespace drone_mapper;
    DroneConfigData d{};
    d.radius = radius_cm * cm;
    return d;
}

// Builds a 1 cm/voxel world: a sealed 3x3x3 Empty chamber (cells [1..3]^3) inside an Occupied shell,
// with a single 1-voxel exit carved at (4,2,2) opening onto Unmapped space (cells 5..6). Bounds are
// [0..6] so exploration terminates. A point/small drone can slip out through the exit; a drone whose
// radius spans a whole voxel cannot fit the gap (its body would overlap the shell).
TestMap makeChamberWithNarrowExit() {
    using V = drone_mapper::types::VoxelOccupancy;
    TestMap m{1.0};
    m.setGridBounds(0, 6);
    for (int x = 0; x <= 4; ++x)
        for (int y = 0; y <= 4; ++y)
            for (int z = 0; z <= 4; ++z) {
                const bool inside = x >= 1 && x <= 3 && y >= 1 && y <= 3 && z >= 1 && z <= 3;
                m.setVoxel(posAt(x, y, z), inside ? V::Empty : V::Occupied);
            }
    m.setVoxel(posAt(4, 2, 2), V::Empty); // carve the single 1-voxel exit
    return m;
}

// Drives the algorithm from a start cell-centre and records every grid cell the drone occupies.
std::set<std::tuple<int, int, int>> reachedCells(drone_mapper::MappingAlgorithmImpl& algo,
                                                 double x0, double y0, double z0, int cap = 4000) {
    using namespace drone_mapper;
    using T = types::MovementCommandType;
    double x = x0, y = y0, z = z0, heading = 0.0;
    std::set<std::tuple<int, int, int>> cells;
    const auto record = [&] {
        cells.insert({static_cast<int>(std::floor(x)),
                      static_cast<int>(std::floor(y)),
                      static_cast<int>(std::floor(z))});
    };
    record();
    for (int i = 0; i < cap; ++i) {
        const auto cmd = algo.nextStep(stateAt(x, y, z, heading), nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished) break;
        if (cmd.movement) {
            const auto& mv = *cmd.movement;
            if (mv.type == T::Rotate) {
                const double a = mv.angle.numerical_value_in(deg);
                heading += (mv.rotation == types::RotationDirection::Left) ? a : -a;
                while (heading >= 360.0) heading -= 360.0;
                while (heading <    0.0) heading += 360.0;
            } else if (mv.type == T::Advance) {
                const double d = mv.distance.numerical_value_in(cm);
                const double r = heading * std::numbers::pi / 180.0;
                x += d * std::cos(r);
                y += d * std::sin(r);
            } else if (mv.type == T::Elevate) {
                z += mv.distance.numerical_value_in(cm);
            }
        }
        record();
    }
    return cells;
}

// True if any reached cell has grid-x >= threshold (i.e. the drone escaped the chamber).
bool reachedBeyondX(const std::set<std::tuple<int, int, int>>& cells, int threshold) {
    for (const auto& c : cells) {
        if (std::get<0>(c) >= threshold) return true;
    }
    return false;
}

} // namespace

// A point-sized drone (radius 0) navigates out through the 1-voxel exit (baseline / no regression).
TEST_F(MappingAlgorithm, PointDroneExitsThroughOneVoxelGap) {
    TestMap chamber = makeChamberWithNarrowExit();
    MissionConfigData mc{};
    LidarConfigData   lc{};
    DroneConfigData   d = droneWithRadius(0.0);
    drone_mapper::MappingAlgorithmImpl algo{mc, lc, d, chamber};

    const auto cmd = algo.nextStep(stateAt(2.5, 2.5, 2.5, 0.0), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Working);
    EXPECT_TRUE(cmd.movement.has_value()) << "a point drone must plan a move toward the exit";
}

// A small drone (radius 0.5 cm = 1-voxel diameter) still fits the 1-voxel exit and plans to leave.
TEST_F(MappingAlgorithm, SmallDroneExitsThroughOneVoxelGap) {
    TestMap chamber = makeChamberWithNarrowExit();
    MissionConfigData mc{};
    LidarConfigData   lc{};
    DroneConfigData   d = droneWithRadius(0.5);
    drone_mapper::MappingAlgorithmImpl algo{mc, lc, d, chamber};

    const auto cmd = algo.nextStep(stateAt(2.5, 2.5, 2.5, 0.0), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Working);
    ASSERT_TRUE(cmd.movement.has_value());
}

// A large drone (radius 1.0 cm = 2-voxel diameter) cannot fit the 1-voxel exit, nor even move to a
// chamber-edge cell (its body would overlap the shell), so the planner finds no navigable frontier
// and returns Finished without commanding a move.
TEST_F(MappingAlgorithm, LargeDroneCannotExitThroughOneVoxelGap) {
    TestMap chamber = makeChamberWithNarrowExit();
    MissionConfigData mc{};
    LidarConfigData   lc{};
    DroneConfigData   d = droneWithRadius(1.0);
    drone_mapper::MappingAlgorithmImpl algo{mc, lc, d, chamber};

    const auto cmd = algo.nextStep(stateAt(2.5, 2.5, 2.5, 0.0), nullptr);
    EXPECT_EQ(cmd.status, drone_mapper::types::AlgorithmStatus::Finished);
    EXPECT_FALSE(cmd.movement.has_value());
}

// End-to-end on the same world: the small drone escapes the chamber (reaches cells beyond the exit),
// the large drone does not — different drone sizes yield different reachable areas.
TEST_F(MappingAlgorithm, DifferentDroneSizesReachDifferentAreas) {
    MissionConfigData mc{};
    LidarConfigData   lc{};

    TestMap chamber_small = makeChamberWithNarrowExit();
    DroneConfigData d_small = droneWithRadius(0.5);
    drone_mapper::MappingAlgorithmImpl algo_small{mc, lc, d_small, chamber_small};
    const auto reached_small = reachedCells(algo_small, 2.5, 2.5, 2.5);

    TestMap chamber_large = makeChamberWithNarrowExit();
    DroneConfigData d_large = droneWithRadius(1.0);
    drone_mapper::MappingAlgorithmImpl algo_large{mc, lc, d_large, chamber_large};
    const auto reached_large = reachedCells(algo_large, 2.5, 2.5, 2.5);

    EXPECT_TRUE(reachedBeyondX(reached_small, 4)) << "small drone should escape the chamber";
    EXPECT_FALSE(reachedBeyondX(reached_large, 4)) << "large drone must stay confined";
    EXPECT_GT(reached_small.size(), reached_large.size());
}
