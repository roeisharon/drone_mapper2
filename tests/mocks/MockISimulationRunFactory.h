#pragma once

#include <drone_mapper/simulation/ISimulationRunFactory.h>
#include <gmock/gmock.h>
#include <memory>

namespace drone_mapper::mocks {

// GMock wrapper for ISimulationRunFactory.
// Inject into SimulationManager tests to return scripted MockISimulationRun objects
// and verify that create() is called the correct number of times with the right configs.
class MockISimulationRunFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>, create,
                (const types::SimulationConfigData& simulation,
                 const types::MissionConfigData&    mission,
                 const types::DroneConfigData&      drone,
                 const types::LidarConfigData&      lidar,
                 const std::filesystem::path&       output_path),
                (override));
};

}  // namespace drone_mapper::mocks
