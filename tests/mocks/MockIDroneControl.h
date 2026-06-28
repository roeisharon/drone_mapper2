#pragma once

#include <drone_mapper/drone/IDroneControl.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for IDroneControl.
// Inject into MissionControl tests to control what each step returns
// (Continue, Completed, Error) without running real sensor or movement logic.
class MockIDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step,  (),       (override));
    MOCK_METHOD(types::DroneState,      state, (), (const, override));
};

}  // namespace drone_mapper::mocks
