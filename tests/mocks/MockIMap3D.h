#pragma once

#include <drone_mapper/map/IMap3D.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for IMap3D (read-only map view).
// Inject into MapsComparison and SimulationRun tests to return scripted voxel
// occupancy values and map configs without loading a real .npy file.
class MockIMap3D : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel,    (const Position3D& pos), (const, override));
    MOCK_METHOD(types::MapConfig,      getMapConfig, (),                    (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D& pos), (const, override));
};

}  // namespace drone_mapper::mocks
