#pragma once

#include <drone_mapper/mocks/IGPS.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for IGPS.
// Inject into DroneControl tests to provide controlled position and heading
// without a real GPS device or MockGPS instance.
class MockIGPS : public IGPS {
public:
    MOCK_METHOD(Position3D,  position, (), (const, override));
    MOCK_METHOD(Orientation, heading,  (), (const, override));
};

}  // namespace drone_mapper::mocks
