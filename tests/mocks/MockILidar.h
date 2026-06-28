#pragma once

#include <drone_mapper/mocks/ILidar.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for ILidar.
// Inject into DroneControl tests to return controlled scan results
// without running real ray-marching geometry.
class MockILidar : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation scan_orientation), (const, override));
    MOCK_METHOD(types::LidarConfigData, config, (), (const, override));
};

}  // namespace drone_mapper::mocks
