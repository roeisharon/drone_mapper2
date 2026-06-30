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

    // Snaps a world position to the centre of the grid cell that contains it, under the given map
    // config (resolution + offset). The drone starts here so its position coincides with a planner
    // grid-cell centre — the first move then lands cleanly on the grid instead of drifting from a
    // sub-cell origin (Checkpoint A). Pure + static for testing.
    [[nodiscard]] static Position3D
    snapToCellCenter(const Position3D& position, const types::MapConfig& config);
};

} // namespace drone_mapper
