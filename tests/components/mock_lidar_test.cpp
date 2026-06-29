#include <gtest/gtest.h>

#include <drone_mapper/mocks/MockGPS.h>
#include <drone_mapper/mocks/MockLidar.h>
#include <drone_mapper/map/Map3DImpl.h>
#include <drone_mapper/Units.h>

#include <TinyNPY.h>

#include <limits>
#include <memory>

// The fixture is at global scope and named "MockLidar" so --gtest_filter=MockLidar.* routes
// here; production types use the fully-qualified drone_mapper:: prefix to avoid shadowing.

namespace {

// Builds a cubic output map (all Unmapped) with a uniform resolution.
drone_mapper::Map3DImpl makeMap(double side_cm, double res_cm) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = res_cm * cm;
    cfg.offset     = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    cfg.boundaries = {
        0.0 * x_extent[cm], side_cm * x_extent[cm],
        0.0 * y_extent[cm], side_cm * y_extent[cm],
        0.0 * z_extent[cm], side_cm * z_extent[cm],
    };
    return Map3DImpl(std::make_shared<NpyArray>(), cfg);
}

// Expected total beams for n FOV circles: circle 0 has 1 beam, circle k has 4^k. Total = 1+4+...
std::size_t expectedBeams(std::size_t fov_circles) {
    std::size_t total = 0;
    std::size_t count = 1;
    for (std::size_t i = 0; i < fov_circles; ++i) {
        total += count;
        count *= 4;
    }
    return total;
}

} // namespace

// Provides a shared map and GPS; each test builds its own MockLidar with a chosen config.
class MockLidar : public ::testing::Test {
protected:
    static constexpr double kSide = 200.0; // map side length in cm
    static constexpr double kRes  = 10.0;  // map resolution in cm/voxel

    drone_mapper::Map3DImpl map_{makeMap(kSide, kRes)};
    drone_mapper::MockGPS   gps_{{0.0 * drone_mapper::x_extent[drone_mapper::cm],
                                  0.0 * drone_mapper::y_extent[drone_mapper::cm],
                                  50.0 * drone_mapper::z_extent[drone_mapper::cm]},
                                 {0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg},
                                 10.0 * drone_mapper::cm}; // GPS resolution (new 3-arg ctor)

    // Builds a LidarConfigData with the given FOV circle count and beam range.
    static drone_mapper::types::LidarConfigData makeCfg(std::size_t circles,
                                                        double z_max_cm = 150.0) {
        return {
            10.0 * drone_mapper::cm,      // z_min: minimum detection range
            z_max_cm * drone_mapper::cm,  // z_max: maximum beam range
            10.0 * drone_mapper::cm,      // d: ring spacing between FOV circles
            circles,
        };
    }
};

// 0 FOV circles fires no beams → empty scan.
TEST_F(MockLidar, ZeroFovCirclesReturnsEmptyScan) {
    drone_mapper::MockLidar lidar(makeCfg(0), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});
    EXPECT_TRUE(result.empty());
}

// 1 FOV circle fires only the center beam → exactly 1 hit.
TEST_F(MockLidar, OneFovCircleReturnsOneCenterBeam) {
    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});
    EXPECT_EQ(result.size(), 1u);
}

// 2 FOV circles: circle 0 (1) + circle 1 (4) = 5 beams.
TEST_F(MockLidar, TwoFovCirclesReturnFiveBeams) {
    drone_mapper::MockLidar lidar(makeCfg(2), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});
    EXPECT_EQ(result.size(), expectedBeams(2));
}

// An Occupied voxel directly ahead is detected at ~its distance (within one voxel).
TEST_F(MockLidar, CenterBeamHitsOccupiedVoxelAtCorrectDistance) {
    map_.set({50.0 * drone_mapper::x_extent[drone_mapper::cm],
               0.0 * drone_mapper::y_extent[drone_mapper::cm],
              50.0 * drone_mapper::z_extent[drone_mapper::cm]},
             drone_mapper::types::VoxelOccupancy::Occupied);

    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});

    ASSERT_EQ(result.size(), 1u);
    const double dist = result[0].distance.numerical_value_in(drone_mapper::cm);
    EXPECT_GT(dist, 0.0);
    EXPECT_LT(dist, std::numeric_limits<double>::max() / 2.0);
    EXPECT_NEAR(dist, 50.0, kRes); // beam steps at 0.1×res = 1 cm; within one voxel of 50 cm
}

// With no obstacle in range, the beam exits at z_max and returns the miss sentinel (~max double).
TEST_F(MockLidar, CenterBeamInOpenSpaceReturnsMissDistance) {
    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});

    ASSERT_EQ(result.size(), 1u);
    const double dist = result[0].distance.numerical_value_in(drone_mapper::cm);
    EXPECT_GE(dist, std::numeric_limits<double>::max() / 2.0);
}

// A hit closer than z_min cannot be ranged: the beam reports distance 0 (the "too close" sentinel
// ScanResultToVoxels later treats as PotentiallyOccupied). Here the drone's own voxel is Occupied.
TEST_F(MockLidar, HitBeforeZMinReturnsZeroDistance) {
    map_.set({0.0 * drone_mapper::x_extent[drone_mapper::cm],
              0.0 * drone_mapper::y_extent[drone_mapper::cm],
              50.0 * drone_mapper::z_extent[drone_mapper::cm]},
             drone_mapper::types::VoxelOccupancy::Occupied);

    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0].distance.numerical_value_in(drone_mapper::cm), 0.0);
}

// The angle stored in each hit is the scan_orientation offset (relative to heading), NOT the
// GPS-absolute direction — ScanResultToVoxels adds the drone heading separately.
TEST_F(MockLidar, BeamAngleIsStoredRelativeToDroneHeading) {
    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps_);
    const drone_mapper::Orientation scan_offset{30.0 * drone_mapper::deg, 0.0 * drone_mapper::deg};
    const auto result = lidar.scan(scan_offset);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].angle.horizontal.numerical_value_in(drone_mapper::deg), 30.0, 1e-6);
}

// 3 FOV circles: circle 0 (1) + circle 1 (4) + circle 2 (16) = 21 beams. Confirms the beam count
// grows geometrically (4^k per ring), not just for the 2-circle case.
TEST_F(MockLidar, ThreeFovCirclesReturn21Beams) {
    drone_mapper::MockLidar lidar(makeCfg(3), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});
    EXPECT_EQ(result.size(), expectedBeams(3));
    EXPECT_EQ(result.size(), 21u);
}

// An obstacle at exactly z_max is still detected — the trace loop's upper bound is inclusive
// (distance <= z_max). Drone at x=0 facing +x; occupied voxel at x=150 cm with z_max=150 cm.
TEST_F(MockLidar, HitExactlyAtZMaxIsDetected) {
    map_.set({150.0 * drone_mapper::x_extent[drone_mapper::cm],
                0.0 * drone_mapper::y_extent[drone_mapper::cm],
               50.0 * drone_mapper::z_extent[drone_mapper::cm]},
             drone_mapper::types::VoxelOccupancy::Occupied);

    drone_mapper::MockLidar lidar(makeCfg(1, 150.0), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});

    ASSERT_EQ(result.size(), 1u);
    const double dist = result[0].distance.numerical_value_in(drone_mapper::cm);
    EXPECT_LT(dist, std::numeric_limits<double>::max() / 2.0);
    EXPECT_NEAR(dist, 150.0, kRes);
}

// An obstacle just beyond z_max is never reached — the beam truncates at z_max and reports a miss.
// Occupied voxel at x=160 cm lies outside the 150 cm range.
TEST_F(MockLidar, ObstacleBeyondZMaxIsMissed) {
    map_.set({160.0 * drone_mapper::x_extent[drone_mapper::cm],
                0.0 * drone_mapper::y_extent[drone_mapper::cm],
               50.0 * drone_mapper::z_extent[drone_mapper::cm]},
             drone_mapper::types::VoxelOccupancy::Occupied);

    drone_mapper::MockLidar lidar(makeCfg(1, 150.0), map_, gps_);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_GE(result[0].distance.numerical_value_in(drone_mapper::cm),
              std::numeric_limits<double>::max() / 2.0);
}

// A negative scan offset is stored verbatim on the center beam (no wrap to a positive angle):
// the relative angle is the caller's offset, sign preserved.
TEST_F(MockLidar, CenterBeamAngleSignPreservedForNegativeOffset) {
    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps_);
    const drone_mapper::Orientation scan_offset{-45.0 * drone_mapper::deg, 0.0 * drone_mapper::deg};
    const auto result = lidar.scan(scan_offset);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].angle.horizontal.numerical_value_in(drone_mapper::deg), -45.0, 1e-6);
}

// The altitude component of the scan offset is carried through onto the center beam's stored angle.
TEST_F(MockLidar, CenterBeamStoresAltitudeOffset) {
    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps_);
    const drone_mapper::Orientation scan_offset{0.0 * drone_mapper::deg, 20.0 * drone_mapper::deg};
    const auto result = lidar.scan(scan_offset);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].angle.altitude.numerical_value_in(drone_mapper::deg), 20.0, 1e-6);
}

// GPS heading steers the traced beam, not just the stored angle. With the drone rotated to face +y
// (heading 90 deg) and a zero scan offset, an obstacle along +y is detected even though a heading-0
// drone would travel +x and miss it.
TEST_F(MockLidar, HeadingRotatesAbsoluteBeamDirection) {
    map_.set({  0.0 * drone_mapper::x_extent[drone_mapper::cm],
               50.0 * drone_mapper::y_extent[drone_mapper::cm],
               50.0 * drone_mapper::z_extent[drone_mapper::cm]},
             drone_mapper::types::VoxelOccupancy::Occupied);

    drone_mapper::MockGPS gps90{{0.0 * drone_mapper::x_extent[drone_mapper::cm],
                                 0.0 * drone_mapper::y_extent[drone_mapper::cm],
                                 50.0 * drone_mapper::z_extent[drone_mapper::cm]},
                                {90.0 * drone_mapper::deg, 0.0 * drone_mapper::deg},
                                10.0 * drone_mapper::cm};
    drone_mapper::MockLidar lidar(makeCfg(1), map_, gps90);
    const auto result = lidar.scan({0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg});

    ASSERT_EQ(result.size(), 1u);
    const double dist = result[0].distance.numerical_value_in(drone_mapper::cm);
    EXPECT_LT(dist, std::numeric_limits<double>::max() / 2.0);
    EXPECT_NEAR(dist, 50.0, kRes);
}

// Lidar config affects coverage: with the same obstacle, a long-range config detects it while a
// short-range config (smaller z_max) does not. Demonstrates that swapping lidar configs changes the
// observed map rather than being ignored.
TEST_F(MockLidar, RangeConfigChangesObstacleDetectionCoverage) {
    using namespace drone_mapper;
    // Obstacle straight ahead (+X) at ~105 cm; the fixture GPS sits at (0,0,50) facing 0deg.
    map_.set({105.0 * x_extent[cm], 0.0 * y_extent[cm], 50.0 * z_extent[cm]},
             types::VoxelOccupancy::Occupied);

    drone_mapper::MockLidar long_range(makeCfg(1, 150.0), map_, gps_); // z_max 150 -> reaches it
    drone_mapper::MockLidar short_range(makeCfg(1, 50.0), map_, gps_);  // z_max 50  -> stops short

    const auto long_scan  = long_range.scan({0.0 * deg, 0.0 * deg});
    const auto short_scan = short_range.scan({0.0 * deg, 0.0 * deg});

    ASSERT_EQ(long_scan.size(), 1u);  // 1 FOV circle -> a single centre beam
    ASSERT_EQ(short_scan.size(), 1u);
    const double miss = std::numeric_limits<double>::max();
    EXPECT_LT(long_scan[0].distance.numerical_value_in(cm), miss / 2.0)
        << "long-range lidar should detect the obstacle";
    EXPECT_DOUBLE_EQ(short_scan[0].distance.numerical_value_in(cm), miss)
        << "short-range lidar should miss the out-of-range obstacle";
}

// config() (added 20.6) returns the LidarConfigData the lidar was constructed with, field for field.
TEST_F(MockLidar, ConfigReturnsInjectedConfig) {
    const drone_mapper::types::LidarConfigData cfg = makeCfg(4, 200.0);
    drone_mapper::MockLidar lidar(cfg, map_, gps_);
    const auto out = lidar.config();
    EXPECT_DOUBLE_EQ(out.z_min.numerical_value_in(drone_mapper::cm),
                     cfg.z_min.numerical_value_in(drone_mapper::cm));
    EXPECT_DOUBLE_EQ(out.z_max.numerical_value_in(drone_mapper::cm),
                     cfg.z_max.numerical_value_in(drone_mapper::cm));
    EXPECT_DOUBLE_EQ(out.d.numerical_value_in(drone_mapper::cm),
                     cfg.d.numerical_value_in(drone_mapper::cm));
    EXPECT_EQ(out.fov_circles, cfg.fov_circles);
}
