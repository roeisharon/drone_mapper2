#pragma once

#include <TinyNPY.h>

#include <drone_mapper/map/IMutableMap3D.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    // Changed: added offset-aware construction for hidden maps loaded from NPY files.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config);

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    // Changed: exposes boundaries, offset, and resolution as one map-owned configuration.
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    //Mutable map methods
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    // Maps a world position to a flat array index, or nullopt when out of bounds.
    // Shared by atVoxel, set, and isInBounds so the index math lives in one place.
    [[nodiscard]] std::optional<std::size_t> flatIndexFor(const Position3D& pos) const;

    std::shared_ptr<NpyArray> map_;
    types::MapConfig config_;
    // Voxel occupancy as int8 (one byte per voxel), row-major (C) order.
    // int8 round-trips every VoxelOccupancy value, including PotentiallyOccupied (-3).
    std::vector<std::int8_t> data_;
    std::size_t x_size_ = 0;
    std::size_t y_size_ = 0;
    std::size_t z_size_ = 0;
};

} // namespace drone_mapper
