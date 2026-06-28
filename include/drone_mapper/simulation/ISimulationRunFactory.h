#pragma once

#include <drone_mapper/simulation/ISimulationRun.h>
#include <drone_mapper/Types.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

// **Do not change this interface.**
class ISimulationRunFactory {
public:
    virtual ~ISimulationRunFactory() = default;

    [[nodiscard]] virtual std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) = 0;
};

} // namespace drone_mapper
