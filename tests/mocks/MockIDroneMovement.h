#pragma once

#include <drone_mapper/mocks/IDroneMovement.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for IDroneMovement.
// Inject into DroneControl tests to verify clamping behaviour and to simulate
// movement failures (e.g. return success=false) without touching real GPS state.
class MockIDroneMovement : public IDroneMovement {
public:
    MOCK_METHOD(types::MovementResult, rotate,
                (types::RotationDirection direction, HorizontalAngle angle), (override));
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength distance), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength distance), (override));
};

}  // namespace drone_mapper::mocks
