#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/utils/MapsComparison.h>
#include <drone_mapper/map/Map3DImpl.h>
#include <drone_mapper/Units.h>

#include "mocks/MockIMap3D.h"

#include <TinyNPY.h>

#include <memory>
#include <vector>

namespace {

using drone_mapper::cm;
using drone_mapper::x_extent;
using drone_mapper::y_extent;
using drone_mapper::z_extent;
using drone_mapper::Position3D;
using drone_mapper::types::VoxelOccupancy;
using drone_mapper::types::MappingBounds;
using drone_mapper::types::MapConfig;

// Build an n_x x n_y x n_z output-style map at 1 cm/voxel, no offset. Boundaries are set
// explicitly because compare() iterates them from origin.getMapConfig(); an empty NpyArray means
// every voxel starts Unmapped until set().
std::unique_ptr<drone_mapper::Map3DImpl> makeMap(int n_x, int n_y, int n_z) {
    const MapConfig cfg{
        MappingBounds{
            0.0 * x_extent[cm], static_cast<double>(n_x) * x_extent[cm],
            0.0 * y_extent[cm], static_cast<double>(n_y) * y_extent[cm],
            0.0 * z_extent[cm], static_cast<double>(n_z) * z_extent[cm]
        },
        Position3D{},
        1.0 * cm
    };
    return std::make_unique<drone_mapper::Map3DImpl>(std::make_shared<NpyArray>(), cfg);
}

// Write val at grid index (ix, iy, iz), addressing the voxel centre so atVoxel() is unambiguous.
void setVoxel(drone_mapper::Map3DImpl& m, int ix, int iy, int iz, VoxelOccupancy val) {
    m.set({(static_cast<double>(ix) + 0.5) * x_extent[cm],
           (static_cast<double>(iy) + 0.5) * y_extent[cm],
           (static_cast<double>(iz) + 0.5) * z_extent[cm]},
          val);
}

// Fill every voxel of an n_x x n_y x n_z map with one value.
void fillAll(drone_mapper::Map3DImpl& m, int n_x, int n_y, int n_z, VoxelOccupancy val) {
    for (int x = 0; x < n_x; ++x)
        for (int y = 0; y < n_y; ++y)
            for (int z = 0; z < n_z; ++z)
                setVoxel(m, x, y, z, val);
}

} // namespace

// Basic score correctness

// Two identical all-Empty maps score 100.
TEST(MapsComparison, IdenticalMapsGive100) {
    auto origin = makeMap(3, 3, 1);
    auto target = makeMap(3, 3, 1);
    fillAll(*origin, 3, 3, 1, VoxelOccupancy::Empty);
    fillAll(*target, 3, 3, 1, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

// A fully-Unmapped target matches no origin voxel and scores 0.
TEST(MapsComparison, FullyUnmappedTargetGives0) {
    auto origin = makeMap(3, 3, 1);
    auto target = makeMap(3, 3, 1);
    fillAll(*origin, 3, 3, 1, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

// Half-matching target scores proportionally (3 of 6 mappable → 50).
TEST(MapsComparison, PartiallyMappedTargetCorrectScore) {
    auto origin = makeMap(3, 2, 1);
    auto target = makeMap(3, 2, 1);
    fillAll(*origin, 3, 2, 1, VoxelOccupancy::Empty);
    setVoxel(*target, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*target, 1, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*target, 2, 0, 0, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 50.0);
}

// An all-Unmapped origin has zero mappable voxels → score 0 (no division by zero).
TEST(MapsComparison, OriginAllUnmappedGives0) {
    auto origin = makeMap(3, 3, 1);
    auto target = makeMap(3, 3, 1);
    fillAll(*target, 3, 3, 1, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

// Observable Occupied voxels (adjacent to an Empty voxel) that match correctly score 100.
// In a 3x1x1 map [Empty, Occupied, Occupied], only [0] and [1] are observable:
// [0] is Empty; [1] has an Empty neighbor at [0]; [2] is interior (no Empty neighbor).
TEST(MapsComparison, ObservableSurfaceOccupiedMatchesGives100) {
    auto origin = makeMap(3, 1, 1);
    auto target = makeMap(3, 1, 1);
    setVoxel(*origin, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*origin, 1, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*origin, 2, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*target, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*target, 1, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*target, 2, 0, 0, VoxelOccupancy::Occupied);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

// Occupied origin vs wrong-value target: both observable slots wrong → 0.
// 2x1x1: origin=[Empty, Occupied], target=[Occupied, Empty]; both voxels mismatch.
TEST(MapsComparison, AllObservableVoxelsMismatchGives0) {
    auto origin = makeMap(2, 1, 1);
    auto target = makeMap(2, 1, 1);
    setVoxel(*origin, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*origin, 1, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*target, 0, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*target, 1, 0, 0, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

// Mixed origin values are each compared by exact value (1 of 2 matches → 50).
TEST(MapsComparison, MixedOriginCorrectScore) {
    auto origin = makeMap(2, 1, 1);
    auto target = makeMap(2, 1, 1);
    setVoxel(*origin, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*origin, 1, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*target, 0, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*target, 1, 0, 0, VoxelOccupancy::Occupied);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 50.0);
}

// Origin skip behaviour

// Unmapped origin voxels are excluded from the denominator (and not rewarded in the target).
TEST(MapsComparison, UnmappedOriginVoxelsNotCounted) {
    auto origin = makeMap(3, 1, 1);
    auto target = makeMap(3, 1, 1);
    setVoxel(*origin, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*origin, 1, 0, 0, VoxelOccupancy::Empty);
    fillAll(*target, 3, 1, 1, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

// PotentiallyOccupied scoring policy: exact-match. Where origin is Empty, a target
// PotentiallyOccupied is a miss; where origin is PotentiallyOccupied it matches.
// 2x1x1: origin=[Empty, PotentiallyOccupied], target=[PO, PO] → 1 of 2 matches → 50.
// (The interior-exclusion only applies to Occupied voxels, so PO in origin is always counted.)
TEST(MapsComparison, PotentiallyOccupiedHandledPerPolicy) {
    auto origin = makeMap(2, 1, 1);
    auto target = makeMap(2, 1, 1);
    setVoxel(*origin, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*origin, 1, 0, 0, VoxelOccupancy::PotentiallyOccupied);
    setVoxel(*target, 0, 0, 0, VoxelOccupancy::PotentiallyOccupied);
    setVoxel(*target, 1, 0, 0, VoxelOccupancy::PotentiallyOccupied);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 50.0);
}

// A zero-resolution origin config must not loop forever — compare() guards and returns 0.
TEST(MapsComparison, ZeroResolutionOriginReturnsZeroScores) {
    using namespace ::testing;
    NiceMock<drone_mapper::mocks::MockIMap3D> origin_mock;
    const MapConfig zero_res_cfg{MappingBounds{}, Position3D{}, 0.0 * cm};
    ON_CALL(origin_mock, getMapConfig()).WillByDefault(Return(zero_res_cfg));
    NiceMock<drone_mapper::mocks::MockIMap3D> target_mock;

    const auto scores = drone_mapper::MapsComparison::compare(origin_mock, {&target_mock});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

// Multiple targets

// An empty target vector yields an empty score vector.
TEST(MapsComparison, EmptyTargetVectorReturnsEmpty) {
    auto origin = makeMap(3, 3, 1);
    fillAll(*origin, 3, 3, 1, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {});
    EXPECT_TRUE(scores.empty());
}

// Two targets produce exactly two scores.
TEST(MapsComparison, TwoTargetsReturnTwoScores) {
    auto origin  = makeMap(3, 3, 1);
    auto target1 = makeMap(3, 3, 1);
    auto target2 = makeMap(3, 3, 1);
    fillAll(*origin, 3, 3, 1, VoxelOccupancy::Empty);
    fillAll(*target1, 3, 3, 1, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target1.get(), target2.get()});
    EXPECT_EQ(scores.size(), 2u);
}

// Each target is scored independently against the same origin (100 and 0).
TEST(MapsComparison, TwoTargetsScoredIndependently) {
    auto origin  = makeMap(4, 1, 1);
    auto target1 = makeMap(4, 1, 1);
    auto target2 = makeMap(4, 1, 1);
    fillAll(*origin, 4, 1, 1, VoxelOccupancy::Empty);
    fillAll(*target1, 4, 1, 1, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target1.get(), target2.get()});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
}

// Two partially-matching targets get distinct proportional scores (75 and 25).
TEST(MapsComparison, TwoTargetsBothPartiallyMatch) {
    auto origin  = makeMap(4, 1, 1);
    auto target1 = makeMap(4, 1, 1);
    auto target2 = makeMap(4, 1, 1);
    fillAll(*origin, 4, 1, 1, VoxelOccupancy::Empty);
    setVoxel(*target1, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*target1, 1, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*target1, 2, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*target2, 3, 0, 0, VoxelOccupancy::Empty);

    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target1.get(), target2.get()});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 75.0);
    EXPECT_DOUBLE_EQ(scores[1], 25.0);
}

// Correctness against real .npy data

// A real fixture map compared with itself scores 100.
TEST(MapsComparison, RealNpySelfCompareGives100) {
    const auto npy = std::make_shared<NpyArray>();
    const char* err = npy->LoadNPY("data_maps/single_voxel_x4_y4_z4.npy");
    ASSERT_EQ(err, nullptr) << "Failed to load fixture: " << (err ? err : "null");

    const drone_mapper::types::MapConfig cfg{
        drone_mapper::types::MappingBounds{}, drone_mapper::Position3D{}, 1.0 * cm};
    drone_mapper::Map3DImpl map_a{npy, cfg};
    drone_mapper::Map3DImpl map_b{npy, cfg};

    const auto scores = drone_mapper::MapsComparison::compare(map_a, {&map_b});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

// A real fixture origin vs an all-Unmapped output map scores 0.
TEST(MapsComparison, RealNpyAllUnmappedTargetLowScore) {
    const auto npy = std::make_shared<NpyArray>();
    ASSERT_EQ(npy->LoadNPY("data_maps/single_voxel_x4_y4_z4.npy"), nullptr);

    const drone_mapper::types::MapConfig cfg{
        drone_mapper::types::MappingBounds{}, drone_mapper::Position3D{}, 1.0 * cm};
    drone_mapper::Map3DImpl origin{npy, cfg};
    drone_mapper::Map3DImpl empty_target{std::make_shared<NpyArray>(), origin.getMapConfig()};

    const auto scores = drone_mapper::MapsComparison::compare(origin, {&empty_target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

// Interior Occupied voxels (no Empty neighbor in origin) must not count against the score.
// 3x3x3 cube: hollow interior — outer shell Occupied, inner voxel also Occupied (interior).
// Only shell voxels adjacent to the centre Empty region are observable; the truly interior
// voxels (the 1x1x1 core surrounded by Occupied on all sides) must be excluded.
// Verifies that a perfectly-mapped target still reaches 100 despite the interior voxels.
TEST(MapsComparison, InteriorOccupiedVoxelsExcludedFromScoring) {
    // 3x1x1: [Empty, Occupied, Occupied]. Occupied[1] has Empty neighbour → observable.
    // Occupied[2] has only Occupied/OOB neighbours → interior → excluded.
    auto origin = makeMap(3, 1, 1);
    auto target = makeMap(3, 1, 1);
    setVoxel(*origin, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*origin, 1, 0, 0, VoxelOccupancy::Occupied);
    setVoxel(*origin, 2, 0, 0, VoxelOccupancy::Occupied);
    // Target correctly maps [0] and [1], but leaves [2] Unmapped (as a drone would).
    setVoxel(*target, 0, 0, 0, VoxelOccupancy::Empty);
    setVoxel(*target, 1, 0, 0, VoxelOccupancy::Occupied);

    // [2] is not observable, so total_mappable=2 and both match → 100.
    const auto scores = drone_mapper::MapsComparison::compare(*origin, {target.get()});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}
