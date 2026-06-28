#include <drone_mapper/map/Map3DImpl.h>

#include <TinyNPY.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Fixtures are at global scope (not inside namespace drone_mapper) so the fixture class names
// (Map3D, Map3DOffset, …) do not shadow production types; production types are fully-qualified
// or pulled in with "using namespace drone_mapper" inside individual tests.

namespace {

// MapConfig with 1 cm/voxel and zero offset — matches the 5×5×5 fixture files.
drone_mapper::types::MapConfig unitConfig() {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset     = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    return cfg;
}

// Loads a NpyArray from disk and asserts that loading succeeded.
std::shared_ptr<NpyArray> loadNpy(const std::string& path) {
    auto arr = std::make_shared<NpyArray>();
    const char* err = arr->LoadNPY(path.c_str());
    EXPECT_EQ(err, nullptr) << "Failed to load: " << path;
    return arr;
}

// Builds a Position3D from raw cm doubles.
drone_mapper::Position3D pos(double x, double y, double z) {
    using namespace drone_mapper;
    return {x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

// Writes a minimal .npy v1.0 file with the given dtype descriptor and shape (data is zero-filled).
// Used to craft intentionally-invalid maps (wrong rank / wrong element size) for the ctor-validation
// tests, since all data_maps/ fixtures are valid 3-D uint8.
void writeNpy(const std::filesystem::path& path, const std::string& descr,
              const std::vector<std::size_t>& shape, std::size_t bytes_per_value) {
    std::ostringstream dict;
    dict << "{'descr': '" << descr << "', 'fortran_order': False, 'shape': (";
    std::size_t count = 1;
    for (std::size_t i = 0; i < shape.size(); ++i) {
        if (i) dict << ", ";
        dict << shape[i];
        count *= shape[i];
    }
    dict << "), }";

    constexpr std::size_t kPrefixLen = 10; // magic(6) + major(1) + minor(1) + hlen(2)
    std::size_t dict_len = dict.str().size() + 1; // +1 for the trailing '\n'
    const std::size_t padded = ((kPrefixLen + dict_len + 63) / 64) * 64;
    dict_len = padded - kPrefixLen;
    std::string header = dict.str();
    header.resize(dict_len - 1, ' ');
    header += '\n';

    std::ofstream out(path, std::ios::binary);
    out.write("\x93NUMPY", 6);
    out.put('\x01');
    out.put('\x00');
    const auto hlen = static_cast<std::uint16_t>(dict_len);
    out.write(reinterpret_cast<const char*>(&hlen), 2);
    out.write(header.data(), static_cast<std::streamsize>(dict_len));
    const std::vector<char> data(count * bytes_per_value, 0);
    out.write(data.data(), static_cast<std::streamsize>(data.size()));
}

// Writes a minimal 3-D uint8 .npy with explicit voxel bytes (row-major, C order). Used to test that
// Minecraft-style block ids (0 = air, any other id = solid) decode correctly.
void writeNpyU8(const std::filesystem::path& path, const std::vector<std::size_t>& shape,
                const std::vector<std::uint8_t>& bytes) {
    std::ostringstream dict;
    dict << "{'descr': '|u1', 'fortran_order': False, 'shape': (";
    for (std::size_t i = 0; i < shape.size(); ++i) {
        if (i) dict << ", ";
        dict << shape[i];
    }
    dict << "), }";

    constexpr std::size_t kPrefixLen = 10;
    std::size_t dict_len = dict.str().size() + 1;
    const std::size_t padded = ((kPrefixLen + dict_len + 63) / 64) * 64;
    dict_len = padded - kPrefixLen;
    std::string header = dict.str();
    header.resize(dict_len - 1, ' ');
    header += '\n';

    std::ofstream out(path, std::ios::binary);
    out.write("\x93NUMPY", 6);
    out.put('\x01');
    out.put('\x00');
    const auto hlen = static_cast<std::uint16_t>(dict_len);
    out.write(reinterpret_cast<const char*>(&hlen), 2);
    out.write(header.data(), static_cast<std::streamsize>(dict_len));
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

// Builds an output-map Map3DImpl: a cubic volume of the given side length and resolution,
// from an empty NpyArray (so storage is sized from the boundaries).
drone_mapper::Map3DImpl makeOutputMap(int side_cm = 50, int res_cm = 10) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = static_cast<double>(res_cm) * cm;
    cfg.offset     = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    cfg.boundaries = {
        0.0 * x_extent[cm], static_cast<double>(side_cm) * x_extent[cm],
        0.0 * y_extent[cm], static_cast<double>(side_cm) * y_extent[cm],
        0.0 * z_extent[cm], static_cast<double>(side_cm) * z_extent[cm],
    };
    return Map3DImpl(std::make_shared<NpyArray>(), cfg);
}

} // namespace

// Loads single_voxel_x2_y4_z2.npy (5×5×5, one Occupied voxel at index [2][4][2]) per test.
class Map3D : public ::testing::Test {
protected:
    static constexpr const char* kSingleVoxelPath = "data_maps/single_voxel_x2_y4_z2.npy";
    std::unique_ptr<drone_mapper::Map3DImpl> map_;

    void SetUp() override {
        map_ = std::make_unique<drone_mapper::Map3DImpl>(loadNpy(kSingleVoxelPath), unitConfig());
    }
};

// Construction from a valid file must not leave the object null.
TEST_F(Map3D, LoadedMapIsNonNull) {
    EXPECT_NE(map_, nullptr);
}

// Index [2][4][2] is the single occupied voxel (world (2,4,2) at 1 cm/voxel).
TEST_F(Map3D, OccupiedVoxelReadCorrectly) {
    EXPECT_EQ(map_->atVoxel(pos(2, 4, 2)), drone_mapper::types::VoxelOccupancy::Occupied);
}

// The origin voxel [0][0][0] is not occupied and must read back as Empty.
TEST_F(Map3D, EmptyVoxelReadCorrectly) {
    EXPECT_EQ(map_->atVoxel(pos(0, 0, 0)), drone_mapper::types::VoxelOccupancy::Empty);
}

// Shape (5,5,5), res 1 cm, offset 0 → boundaries span [0, 5] on each axis.
TEST_F(Map3D, BoundariesDerivedFromShape) {
    const auto cfg = map_->getMapConfig();
    EXPECT_DOUBLE_EQ(cfg.boundaries.min_x.numerical_value_in(drone_mapper::cm),      0.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.max_x.numerical_value_in(drone_mapper::cm),      5.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.min_y.numerical_value_in(drone_mapper::cm),      0.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.max_y.numerical_value_in(drone_mapper::cm),      5.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.min_height.numerical_value_in(drone_mapper::cm), 0.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.max_height.numerical_value_in(drone_mapper::cm), 5.0);
}

// The resolution and offset passed to the constructor must be retrievable unchanged.
TEST_F(Map3D, ResolutionAndOffsetPreservedInConfig) {
    const auto cfg = map_->getMapConfig();
    EXPECT_DOUBLE_EQ(cfg.resolution.numerical_value_in(drone_mapper::cm), 1.0);
    EXPECT_DOUBLE_EQ(cfg.offset.x.numerical_value_in(drone_mapper::cm),  0.0);
    EXPECT_DOUBLE_EQ(cfg.offset.y.numerical_value_in(drone_mapper::cm),  0.0);
    EXPECT_DOUBLE_EQ(cfg.offset.z.numerical_value_in(drone_mapper::cm),  0.0);
}

// A coordinate below the min boundary maps to a negative index → OutOfBounds.
TEST_F(Map3D, NegativePositionIsOutOfBounds) {
    EXPECT_EQ(map_->atVoxel(pos(-1, 0, 0)), drone_mapper::types::VoxelOccupancy::OutOfBounds);
}

// Shape is 5 along x, so world 5.0 maps to index 5 — one past the last valid index.
TEST_F(Map3D, PositionAtShapeEdgeIsOutOfBounds) {
    EXPECT_EQ(map_->atVoxel(pos(5, 0, 0)), drone_mapper::types::VoxelOccupancy::OutOfBounds);
}

// World 4.0 maps to index 4 — the last valid index for a size-5 dimension.
TEST_F(Map3D, LastValidPositionIsInBounds) {
    EXPECT_NE(map_->atVoxel(pos(4, 0, 0)), drone_mapper::types::VoxelOccupancy::OutOfBounds);
}

// isInBounds is true for a valid interior/edge voxel.
TEST_F(Map3D, IsInBoundsTrueForValidVoxel) {
    EXPECT_TRUE(map_->isInBounds(pos(0, 0, 0)));
    EXPECT_TRUE(map_->isInBounds(pos(4, 4, 4)));
}

// isInBounds is false for a negative coordinate.
TEST_F(Map3D, IsInBoundsFalseForNegative) {
    EXPECT_FALSE(map_->isInBounds(pos(-1, 0, 0)));
}

// isInBounds is false one past the last valid index.
TEST_F(Map3D, IsInBoundsFalseAtShapeEdge) {
    EXPECT_FALSE(map_->isInBounds(pos(5, 0, 0)));
}

// isInBounds must agree with atVoxel across the grid: in-bounds iff not OutOfBounds.
TEST_F(Map3D, IsInBoundsAgreesWithAtVoxelOverGrid) {
    for (double x = -1.0; x <= 5.0; x += 1.0) {
        for (double z = -1.0; z <= 5.0; z += 1.0) {
            const auto p = pos(x, 2, z);
            const bool not_oob =
                map_->atVoxel(p) != drone_mapper::types::VoxelOccupancy::OutOfBounds;
            EXPECT_EQ(map_->isInBounds(p), not_oob) << "mismatch at x=" << x << " z=" << z;
        }
    }
}

// Positive offset: world(0,0,0) sits at array index offset/res; array [ix] = world (ix*res - offset).
// With offset (10,20,30), res 1: array [2][4][2] → world (-8,-16,-28); world (2,4,2) → ix 12 → OOB.
TEST(Map3DOffset, OccupiedVoxelAtCorrectWorldPositionWithOffset) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset     = Position3D{10.0 * x_extent[cm], 20.0 * y_extent[cm], 30.0 * z_extent[cm]};
    Map3DImpl m(loadNpy("data_maps/single_voxel_x2_y4_z2.npy"), cfg);

    EXPECT_EQ(m.atVoxel(pos(-8, -16, -28)), types::VoxelOccupancy::Occupied);
    EXPECT_EQ(m.atVoxel(pos( 2,   4,   2)), types::VoxelOccupancy::OutOfBounds);
}

// Boundaries fold in the offset: min = -offset, max = shape*res - offset.
TEST(Map3DOffset, BoundariesIncludeOffset) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset     = Position3D{10.0 * x_extent[cm], 20.0 * y_extent[cm], 30.0 * z_extent[cm]};
    Map3DImpl m(loadNpy("data_maps/single_voxel_x2_y4_z2.npy"), cfg);

    const auto b = m.getMapConfig().boundaries;
    EXPECT_DOUBLE_EQ(b.min_x.numerical_value_in(cm),      -10.0);
    EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(cm),       -5.0);
    EXPECT_DOUBLE_EQ(b.min_y.numerical_value_in(cm),      -20.0);
    EXPECT_DOUBLE_EQ(b.max_y.numerical_value_in(cm),      -15.0);
    EXPECT_DOUBLE_EQ(b.min_height.numerical_value_in(cm), -30.0);
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), -25.0);
}

// isInBounds honors the offset: only the offset-shifted world range is in bounds.
TEST(Map3DOffset, IsInBoundsRespectsOffset) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset     = Position3D{10.0 * x_extent[cm], 20.0 * y_extent[cm], 30.0 * z_extent[cm]};
    Map3DImpl m(loadNpy("data_maps/single_voxel_x2_y4_z2.npy"), cfg);

    EXPECT_TRUE(m.isInBounds(pos(-8, -16, -28)));
    EXPECT_FALSE(m.isInBounds(pos(2, 4, 2)));
}

// Negative offset: logical origin is before the array, so valid world coords are positive.
// offset (-10,-20,-30), res 1: array [2][4][2] → world (12,24,32); world (2,4,2) → ix -8 → OOB.
TEST(Map3DOffset, OccupiedVoxelAtCorrectWorldPositionWithNegativeOffset) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset     = Position3D{-10.0 * x_extent[cm], -20.0 * y_extent[cm], -30.0 * z_extent[cm]};
    Map3DImpl m(loadNpy("data_maps/single_voxel_x2_y4_z2.npy"), cfg);

    EXPECT_EQ(m.atVoxel(pos(12, 24, 32)), types::VoxelOccupancy::Occupied);
    EXPECT_EQ(m.atVoxel(pos( 2,  4,  2)), types::VoxelOccupancy::OutOfBounds);
}

// Boundaries with a negative offset: min = -offset (positive), max = shape*res - offset.
TEST(Map3DOffset, BoundariesWithNegativeOffset) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 1.0 * cm;
    cfg.offset     = Position3D{-10.0 * x_extent[cm], -20.0 * y_extent[cm], -30.0 * z_extent[cm]};
    Map3DImpl m(loadNpy("data_maps/single_voxel_x2_y4_z2.npy"), cfg);

    const auto b = m.getMapConfig().boundaries;
    EXPECT_DOUBLE_EQ(b.min_x.numerical_value_in(cm),      10.0);
    EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(cm),      15.0);
    EXPECT_DOUBLE_EQ(b.min_y.numerical_value_in(cm),      20.0);
    EXPECT_DOUBLE_EQ(b.max_y.numerical_value_in(cm),      25.0);
    EXPECT_DOUBLE_EQ(b.min_height.numerical_value_in(cm), 30.0);
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), 35.0);
}

// A freshly constructed output map has every voxel initialised to Unmapped.
TEST(Map3DOutput, InitialStateIsUnmapped) {
    using namespace drone_mapper;
    auto m = makeOutputMap();
    EXPECT_EQ(m.atVoxel(pos(0,  0,  0)),  types::VoxelOccupancy::Unmapped);
    EXPECT_EQ(m.atVoxel(pos(40, 40, 40)), types::VoxelOccupancy::Unmapped);
}

// set() updates a voxel so a later atVoxel() at the same position returns the written value.
TEST(Map3DOutput, SetThenGetReturnsSetValue) {
    using namespace drone_mapper;
    auto m = makeOutputMap();
    m.set(pos(0, 0, 0), types::VoxelOccupancy::Occupied);
    EXPECT_EQ(m.atVoxel(pos(0, 0, 0)), types::VoxelOccupancy::Occupied);

    m.set(pos(10, 20, 30), types::VoxelOccupancy::Empty);
    EXPECT_EQ(m.atVoxel(pos(10, 20, 30)), types::VoxelOccupancy::Empty);
}

// The new PotentiallyOccupied (-3) value must store and read back faithfully.
TEST(Map3DOutput, SetPotentiallyOccupiedRoundTrips) {
    using namespace drone_mapper;
    auto m = makeOutputMap();
    m.set(pos(10, 10, 10), types::VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(m.atVoxel(pos(10, 10, 10)), types::VoxelOccupancy::PotentiallyOccupied);
}

// A second set() at the same voxel overwrites the first value.
TEST(Map3DOutput, OverwriteVoxelUpdatesValue) {
    using namespace drone_mapper;
    auto m = makeOutputMap();
    m.set(pos(20, 20, 20), types::VoxelOccupancy::Occupied);
    m.set(pos(20, 20, 20), types::VoxelOccupancy::Empty);
    EXPECT_EQ(m.atVoxel(pos(20, 20, 20)), types::VoxelOccupancy::Empty);
}

// Writes outside the map boundary are silently ignored and must not crash or corrupt state.
TEST(Map3DOutput, SetOutOfBoundsDoesNotCrash) {
    using namespace drone_mapper;
    auto m = makeOutputMap();
    EXPECT_NO_THROW(m.set(pos(-10,  0,   0), types::VoxelOccupancy::Occupied));
    EXPECT_NO_THROW(m.set(pos(  0,  0, 999), types::VoxelOccupancy::Occupied));
}

// A map saved and reloaded with the same MapConfig has identical voxel values.
TEST(Map3DSaveReload, SavedMapReloadsWithCorrectValues) {
    using namespace drone_mapper;
    auto original = makeOutputMap();
    original.set(pos(20, 30, 40), types::VoxelOccupancy::Occupied);
    original.set(pos( 0,  0,  0), types::VoxelOccupancy::Empty);

    const auto tmp = std::filesystem::temp_directory_path() / "map3d_save_reload_test.npy";
    original.save(tmp);

    types::MapConfig cfg = original.getMapConfig();
    Map3DImpl reloaded(loadNpy(tmp.string()), cfg);

    EXPECT_EQ(reloaded.atVoxel(pos(20, 30, 40)), types::VoxelOccupancy::Occupied);
    EXPECT_EQ(reloaded.atVoxel(pos( 0,  0,  0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(reloaded.atVoxel(pos(10, 10, 10)), types::VoxelOccupancy::Unmapped);

    std::filesystem::remove(tmp);
}

// PotentiallyOccupied (-3) must survive the .npy save/reload round trip (signed int8 storage).
TEST(Map3DSaveReload, PotentiallyOccupiedSurvivesSaveReload) {
    using namespace drone_mapper;
    auto original = makeOutputMap();
    original.set(pos(30, 30, 30), types::VoxelOccupancy::PotentiallyOccupied);

    const auto tmp = std::filesystem::temp_directory_path() / "map3d_potentially_occupied.npy";
    original.save(tmp);

    Map3DImpl reloaded(loadNpy(tmp.string()), original.getMapConfig());
    EXPECT_EQ(reloaded.atVoxel(pos(30, 30, 30)), types::VoxelOccupancy::PotentiallyOccupied);

    std::filesystem::remove(tmp);
}

// Saving a non-cubic map and reloading it preserves the spatial boundaries (shape round trip).
TEST(Map3DSaveReload, SavedShapeMatchesOriginal) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 10.0 * cm;
    cfg.offset     = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    cfg.boundaries = {
        0.0 * x_extent[cm], 30.0 * x_extent[cm],
        0.0 * y_extent[cm], 40.0 * y_extent[cm],
        0.0 * z_extent[cm], 50.0 * z_extent[cm],
    };
    Map3DImpl original(std::make_shared<NpyArray>(), cfg);

    const auto tmp = std::filesystem::temp_directory_path() / "map3d_shape_test.npy";
    original.save(tmp);

    Map3DImpl reloaded(loadNpy(tmp.string()), cfg);
    const auto b = reloaded.getMapConfig().boundaries;
    EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(cm),      30.0);
    EXPECT_DOUBLE_EQ(b.max_y.numerical_value_in(cm),      40.0);
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), 50.0);

    std::filesystem::remove(tmp);
}

// An empty output map with non-zero boundaries serializes (does not throw) and reloads to a
// non-zero array — guards the regression where output maps saved at zero size.
TEST(Map3DSaveReload, EmptyOutputMapSavesNonZeroSize) {
    using namespace drone_mapper;
    auto m = makeOutputMap(50, 10); // 5×5×5 voxels
    const auto tmp = std::filesystem::temp_directory_path() / "map3d_nonzero_size.npy";

    ASSERT_NO_THROW(m.save(tmp));
    Map3DImpl reloaded(loadNpy(tmp.string()), m.getMapConfig());
    // 5 voxels per axis at 10 cm → boundaries [0,50); a zero-size save would not reload here.
    EXPECT_DOUBLE_EQ(reloaded.getMapConfig().boundaries.max_x.numerical_value_in(cm), 50.0);
    EXPECT_TRUE(reloaded.isInBounds(pos(45, 45, 45)));

    std::filesystem::remove(tmp);
}

// A default-config map (empty array, resolution 0) reports OutOfBounds for every query.
TEST(Map3DDefault, DefaultConfigReturnsOutOfBoundsForAnyPos) {
    using namespace drone_mapper;
    Map3DImpl m(std::make_shared<NpyArray>());
    EXPECT_EQ(m.atVoxel(pos(0, 0, 0)), types::VoxelOccupancy::OutOfBounds);
    EXPECT_FALSE(m.isInBounds(pos(0, 0, 0)));
}

// A null shared_ptr must throw invalid_argument from the constructor.
TEST(Map3DDefault, NullPointerThrows) {
    EXPECT_THROW(drone_mapper::Map3DImpl(nullptr), std::invalid_argument);
}

// A loaded array that is not 3-dimensional must be rejected by the constructor.
TEST(Map3DDefault, NonThreeDimensionalArrayThrows) {
    const auto tmp = std::filesystem::temp_directory_path() / "map3d_bad_rank.npy";
    writeNpy(tmp, "|u1", {2, 2}, 1); // 2-D uint8
    auto arr = std::make_shared<NpyArray>();
    ASSERT_EQ(arr->LoadNPY(tmp.string().c_str()), nullptr);
    EXPECT_THROW((drone_mapper::Map3DImpl(arr)), std::invalid_argument);
    std::filesystem::remove(tmp);
}

// A loaded array with more than one byte per voxel must be rejected by the constructor.
TEST(Map3DDefault, MultiBytePerVoxelArrayThrows) {
    const auto tmp = std::filesystem::temp_directory_path() / "map3d_bad_dtype.npy";
    writeNpy(tmp, "<i4", {2, 2, 2}, 4); // 3-D but 4 bytes per value
    auto arr = std::make_shared<NpyArray>();
    ASSERT_EQ(arr->LoadNPY(tmp.string().c_str()), nullptr);
    EXPECT_THROW((drone_mapper::Map3DImpl(arr)), std::invalid_argument);
    std::filesystem::remove(tmp);
}

// Minecraft-style block ids: 0 = air (Empty), any other block id = solid (Occupied). This guards
// the atVoxel decode for real .npy maps (the staff benchmark map uses ids 1, 3, 4, 18, 45).
TEST(Map3DBlockIds, NonZeroBlockIdsDecodeAsOccupied) {
    using drone_mapper::types::VoxelOccupancy;
    const auto tmp = std::filesystem::temp_directory_path() / "map3d_blockids.npy";
    // A 5x1x1 row: air, dirt(3), leaves(18), brick(45), stone(1).
    writeNpyU8(tmp, {5, 1, 1}, {0, 3, 18, 45, 1});
    drone_mapper::Map3DImpl m{loadNpy(tmp.string()), unitConfig()};
    EXPECT_EQ(m.atVoxel(pos(0, 0, 0)), VoxelOccupancy::Empty);     // air
    EXPECT_EQ(m.atVoxel(pos(1, 0, 0)), VoxelOccupancy::Occupied);  // block id 3
    EXPECT_EQ(m.atVoxel(pos(2, 0, 0)), VoxelOccupancy::Occupied);  // block id 18
    EXPECT_EQ(m.atVoxel(pos(3, 0, 0)), VoxelOccupancy::Occupied);  // block id 45
    EXPECT_EQ(m.atVoxel(pos(4, 0, 0)), VoxelOccupancy::Occupied);  // block id 1
    std::filesystem::remove(tmp);
}

// The staff benchmark map (29x30x31, mixed block ids) parses and reads correctly. Loaded at
// 10 cm/voxel: world extent is (290, 300, 310) cm; the lower half is solid earth (block id 3),
// the top layers are air.
TEST(Map3DBenchmark, OddShapeParsesAndBlocksReadOccupied) {
    using namespace drone_mapper;
    types::MapConfig cfg;
    cfg.resolution = 10.0 * cm;
    cfg.offset     = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    Map3DImpl m{loadNpy("data_maps/benchmark_map.npy"), cfg};

    const auto b = m.getMapConfig().boundaries;
    EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(cm), 290.0);      // 29 * 10
    EXPECT_DOUBLE_EQ(b.max_y.numerical_value_in(cm), 300.0);      // 30 * 10
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), 310.0); // 31 * 10

    // Ground voxel index (10,10,5) = world centre (105,105,55) is solid earth → Occupied.
    EXPECT_EQ(m.atVoxel(pos(105, 105, 55)), types::VoxelOccupancy::Occupied);
    // A voxel high above the roof (z index 29) is air → Empty.
    EXPECT_EQ(m.atVoxel(pos(105, 105, 295)), types::VoxelOccupancy::Empty);
}
