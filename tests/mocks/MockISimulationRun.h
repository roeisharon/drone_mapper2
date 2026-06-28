#pragma once

#include <drone_mapper/simulation/ISimulationRun.h>
#include <gmock/gmock.h>

namespace drone_mapper::mocks {

// GMock wrapper for ISimulationRun.
// Inject into SimulationManager tests to control what each run returns
// without running a real mission.
class MockISimulationRun : public ISimulationRun {
public:
    MOCK_METHOD(types::SimulationResult, run, (), (override));
};

}  // namespace drone_mapper::mocks
