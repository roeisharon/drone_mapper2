#pragma once

#include <drone_mapper/mission/IMissionControl.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for IMissionControl.
// Inject into SimulationRun tests to return a controlled MissionRunResult
// (any status, step count, and error list) without running a real mission loop.
class MockIMissionControl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

}  // namespace drone_mapper::mocks
