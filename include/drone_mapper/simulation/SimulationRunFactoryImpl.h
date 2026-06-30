#pragma once

#include <drone_mapper/simulation/ISimulationRunFactory.h>

namespace drone_mapper {

class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

    // Computes the output map's config from the resolved hidden-map config and the mission bounds.
    // When the mission specifies bounds, the output map is narrowed to (hidden extent ∩ mission
    // bounds) so the drone only plans/scans/moves inside the bounded region; when it doesn't, the
    // output map matches the full hidden extent (unchanged behaviour). Pure + static for testing.
    [[nodiscard]] static types::MapConfig
    outputMapConfig(const types::MapConfig& hidden_config,
                    const types::MappingBounds& mission_bounds);
};

} // namespace drone_mapper
