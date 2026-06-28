#pragma once

#include <drone_mapper/map/IMutableMap3D.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for IMutableMap3D (read-write map view).
// Inject into DroneControl and MissionControl tests to verify that set() and save()
// are called with the correct arguments without writing real .npy files.
class MockIMutableMap3D : public IMutableMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel,    (const Position3D& pos),                        (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (),                                            (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D& pos), (const, override));
    MOCK_METHOD(void,                  set,         (const Position3D& pos, types::VoxelOccupancy value), (override));
    MOCK_METHOD(void,                  save,        (const std::filesystem::path& path),            (const, override));
};

}  // namespace drone_mapper::mocks
