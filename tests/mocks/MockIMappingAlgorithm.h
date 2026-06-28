#pragma once

#include <drone_mapper/drone/IMappingAlgorithm.h>
#include <drone_mapper/map/IMap3D.h>
#include <drone_mapper/Types.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// A minimal read-only IMap3D that always returns Unmapped.
// Used internally by MockIMappingAlgorithm to satisfy the base-class constructor
// requirement without needing the caller to supply a real map.
struct AlwaysUnmappedMap final : public IMap3D {
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D&) const override {
        return types::VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override {
        return {};
    }
    [[nodiscard]] bool isInBounds(const Position3D&) const override {
        return false;
    }
};

// GMock wrapper for IMappingAlgorithm.
// Default-constructible: uses an internal AlwaysUnmappedMap to satisfy the
// base-class constructor.  If a test needs the algorithm to see specific map
// state, construct with an explicit IMap3D reference instead.
class MockIMappingAlgorithm : public IMappingAlgorithm {
    // Inline static: one shared dummy map instance across all MockIMappingAlgorithm objects.
    inline static AlwaysUnmappedMap s_dummy_map_{};

public:
    // Default ctor — uses the shared dummy map with default configs.
    MockIMappingAlgorithm()
        : IMappingAlgorithm(types::MissionConfigData{}, types::LidarConfigData{},
                            types::DroneConfigData{}, s_dummy_map_) {}

    // Explicit ctor — lets a test supply real configs and a map reference.
    MockIMappingAlgorithm(const types::MissionConfigData& mission,
                          const types::LidarConfigData& lidar,
                          const types::DroneConfigData& drone,
                          const IMap3D& map)
        : IMappingAlgorithm(mission, lidar, drone, map) {}

    // nextStep replaces the old nextMove + applyVoxelUpdates pair.
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState& state,
                 const types::LidarScanResult* latest_scan),
                (override));
};

}  // namespace drone_mapper::mocks
