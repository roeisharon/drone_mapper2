#include <gtest/gtest.h>

#include <drone_mapper/utils/ScanResultToVoxels.h>
#include <drone_mapper/map/Map3DImpl.h>
#include <drone_mapper/Units.h>

#include <TinyNPY.h>

#include <limits>
#include <memory>

// Tests the staff-provided ScanResultToVoxels::applyToMap against a real Map3DImpl output map,
// asserting written voxel states by world position. The fixture is at global scope and named
// "ScanResultToVoxels" so --gtest_filter=ScanResultToVoxels.* routes here; the production class
// is referenced fully-qualified (drone_mapper::ScanResultToVoxels) to avoid shadowing.
//
// Geometry is kept crisp: a [0,60) cm cubic output map at 1 cm/voxel (so world coord == array
// index) and a scan origin at (10,10,10) with room for +X/+Y/+Z rays.

using drone_mapper::cm;
using drone_mapper::deg;
using drone_mapper::x_extent;
using drone_mapper::y_extent;
using drone_mapper::z_extent;
using drone_mapper::Position3D;
using drone_mapper::Orientation;
using drone_mapper::Map3DImpl;
using drone_mapper::types::LidarConfigData;
using drone_mapper::types::LidarHit;
using drone_mapper::types::LidarScanResult;
using drone_mapper::types::MapConfig;
using drone_mapper::types::MappingBounds;
using drone_mapper::types::VoxelOccupancy;

namespace {

// A [0, side) cm cubic output map at the given resolution, all voxels Unmapped.
Map3DImpl makeOutputMap(double side_cm = 60.0, double res_cm = 1.0) {
    MapConfig cfg;
    cfg.resolution = res_cm * cm;
    cfg.offset     = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    cfg.boundaries = {
        0.0 * x_extent[cm], side_cm * x_extent[cm],
        0.0 * y_extent[cm], side_cm * y_extent[cm],
        0.0 * z_extent[cm], side_cm * z_extent[cm],
    };
    return Map3DImpl(std::make_shared<NpyArray>(), cfg);
}

Position3D pos(double x, double y, double z) {
    return {x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

// One hit at the given distance and relative beam angle.
LidarScanResult singleHit(double distance_cm, double h_deg = 0.0, double alt_deg = 0.0) {
    return {LidarHit{distance_cm * cm, Orientation{h_deg * deg, alt_deg * deg}}};
}

// A miss: the max-double sentinel MockLidar uses for beams that never return.
LidarScanResult miss(double h_deg = 0.0, double alt_deg = 0.0) {
    return {LidarHit{std::numeric_limits<double>::max() * cm, Orientation{h_deg * deg, alt_deg * deg}}};
}

// A too-close hit: distance 0 means the obstacle was nearer than z_min and can't be ranged.
LidarScanResult zeroHit(double h_deg = 0.0, double alt_deg = 0.0) {
    return {LidarHit{0.0 * cm, Orientation{h_deg * deg, alt_deg * deg}}};
}

} // namespace

class ScanResultToVoxels : public ::testing::Test {
protected:
    Map3DImpl out_{makeOutputMap()};
    Position3D origin_{pos(10, 10, 10)};
    Orientation heading_0_{0.0 * deg, 0.0 * deg};
    Orientation heading_90_{90.0 * deg, 0.0 * deg};
    Orientation heading_45_{45.0 * deg, 0.0 * deg};
    // applyToMap only reads z_min / z_max from the config; d and fov_circles are irrelevant here.
    LidarConfigData cfg_{10.0 * cm, 30.0 * cm, 1.0 * cm, 1};

    void apply(const LidarScanResult& scan, const Orientation& heading) {
        drone_mapper::ScanResultToVoxels::applyToMap(out_, origin_, heading, scan, cfg_);
    }
};

// normal hit -------------------------------------------------------------------

// The path between the origin and a hit is proven free → marked Empty.
TEST_F(ScanResultToVoxels, NormalHitPathBeforeHitMarkedEmpty) {
    apply(singleHit(10.0), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(15, 10, 10)), VoxelOccupancy::Empty); // midway along +X
}

// The hit position itself is marked Occupied.
TEST_F(ScanResultToVoxels, NormalHitVoxelMarkedOccupied) {
    apply(singleHit(10.0), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Occupied);
}

// The hit voxel must be Occupied, not left Empty by the path pass (Occupied wins the merge).
TEST_F(ScanResultToVoxels, NormalHitVoxelIsNotEmpty) {
    apply(singleHit(10.0), heading_0_);
    EXPECT_NE(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Empty);
}

// miss -------------------------------------------------------------------------

// A miss marks the whole ray Empty out to z_max (no obstacle was found in range).
TEST_F(ScanResultToVoxels, MissMarksRayEmptyToZMax) {
    apply(miss(), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(15, 10, 10)), VoxelOccupancy::Empty); // near
    EXPECT_EQ(out_.atVoxel(pos(35, 10, 10)), VoxelOccupancy::Empty); // out near z_max (30)
}

// A miss never marks anything Occupied.
TEST_F(ScanResultToVoxels, MissDoesNotMarkOccupied) {
    apply(miss(), heading_0_);
    EXPECT_NE(out_.atVoxel(pos(35, 10, 10)), VoxelOccupancy::Occupied);
}

// zero distance (too close) ----------------------------------------------------

// A distance-0 hit marks the near segment (origin..z_min) PotentiallyOccupied.
TEST_F(ScanResultToVoxels, ZeroDistanceMarksPotentiallyOccupiedToZMin) {
    apply(zeroHit(), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(15, 10, 10)), VoxelOccupancy::PotentiallyOccupied); // 5 cm < z_min 10
}

// A distance-0 hit never marks anything Occupied (the obstacle position is unknown).
TEST_F(ScanResultToVoxels, ZeroDistanceDoesNotMarkOccupied) {
    apply(zeroHit(), heading_0_);
    EXPECT_NE(out_.atVoxel(pos(15, 10, 10)), VoxelOccupancy::Occupied);
}

// beam geometry ----------------------------------------------------------------

// Heading 0° + relative 0° → beam travels along +X.
TEST_F(ScanResultToVoxels, Heading0BeamTravelsAlongX) {
    apply(singleHit(10.0), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Occupied);
    EXPECT_NE(out_.atVoxel(pos(10, 20, 10)), VoxelOccupancy::Occupied); // not along +Y
}

// Heading 90° + relative 0° → beam travels along +Y.
TEST_F(ScanResultToVoxels, Heading90BeamTravelsAlongY) {
    apply(singleHit(10.0), heading_90_);
    EXPECT_EQ(out_.atVoxel(pos(10, 20, 10)), VoxelOccupancy::Occupied);
    EXPECT_NE(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Occupied); // not along +X
}

// The relative beam angle is added to the drone heading: heading 45° + relative 45° = 90° → +Y.
TEST_F(ScanResultToVoxels, RelativeAngleIsAddedToDroneHeading) {
    apply(singleHit(10.0, /*h_deg=*/45.0), heading_45_);
    EXPECT_EQ(out_.atVoxel(pos(10, 20, 10)), VoxelOccupancy::Occupied);
}

// A positive altitude angle bends the beam upward along +Z.
TEST_F(ScanResultToVoxels, PositiveAltitudeBeamRises) {
    apply(singleHit(10.0, /*h_deg=*/0.0, /*alt_deg=*/90.0), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(10, 10, 20)), VoxelOccupancy::Occupied);
}

// A different scan origin shifts all written voxels accordingly.
TEST_F(ScanResultToVoxels, NonOriginScanOriginOffsetsVoxels) {
    origin_ = pos(30, 30, 30);
    apply(singleHit(10.0), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(40, 30, 30)), VoxelOccupancy::Occupied);
    EXPECT_EQ(out_.atVoxel(pos(35, 30, 30)), VoxelOccupancy::Empty);
}

// priority merge ---------------------------------------------------------------

// Occupied (strongest) overwrites a previously Empty voxel.
TEST_F(ScanResultToVoxels, OccupiedOverwritesEmpty) {
    out_.set(pos(20, 10, 10), VoxelOccupancy::Empty);
    apply(singleHit(10.0), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Occupied);
}

// Empty (from a miss) must NOT overwrite a confirmed Occupied voxel.
TEST_F(ScanResultToVoxels, EmptyDoesNotOverwriteOccupied) {
    out_.set(pos(20, 10, 10), VoxelOccupancy::Occupied);
    apply(miss(), heading_0_); // ray passes through (20,10,10) marking Empty
    EXPECT_EQ(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Occupied);
}

// PotentiallyOccupied (weakest non-zero) must NOT overwrite an Empty voxel.
TEST_F(ScanResultToVoxels, PotentiallyOccupiedDoesNotOverwriteEmpty) {
    out_.set(pos(15, 10, 10), VoxelOccupancy::Empty);
    apply(zeroHit(), heading_0_); // marks PotentiallyOccupied across origin..z_min
    EXPECT_EQ(out_.atVoxel(pos(15, 10, 10)), VoxelOccupancy::Empty);
}

// robustness -------------------------------------------------------------------

// An out-of-bounds scan origin writes nothing (applyToMap returns early).
TEST_F(ScanResultToVoxels, OutOfBoundsScanOriginWritesNothing) {
    origin_ = pos(-50, -50, -50);
    apply(singleHit(10.0), heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(10, 10, 10)), VoxelOccupancy::Unmapped);
    EXPECT_EQ(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Unmapped);
}

// An empty scan (no hits) writes nothing.
TEST_F(ScanResultToVoxels, EmptyScanWritesNothing) {
    apply(LidarScanResult{}, heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(10, 10, 10)), VoxelOccupancy::Unmapped);
    EXPECT_EQ(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Unmapped);
}

// Multiple hits in one scan are each applied independently.
TEST_F(ScanResultToVoxels, MultipleHitsAllApplied) {
    LidarScanResult two{
        LidarHit{10.0 * cm, Orientation{0.0 * deg, 0.0 * deg}},   // +X
        LidarHit{10.0 * cm, Orientation{90.0 * deg, 0.0 * deg}},  // +Y
    };
    apply(two, heading_0_);
    EXPECT_EQ(out_.atVoxel(pos(20, 10, 10)), VoxelOccupancy::Occupied);
    EXPECT_EQ(out_.atVoxel(pos(10, 20, 10)), VoxelOccupancy::Occupied);
}

// A beam that runs off the map edge marks the in-bounds portion and stops without crashing.
TEST_F(ScanResultToVoxels, BeamStopsAtMapBoundary) {
    origin_ = pos(55, 10, 10); // near the +X edge of the [0,60) map
    EXPECT_NO_THROW(apply(miss(), heading_0_));
    EXPECT_EQ(out_.atVoxel(pos(57, 10, 10)), VoxelOccupancy::Empty); // in-bounds part marked
}
