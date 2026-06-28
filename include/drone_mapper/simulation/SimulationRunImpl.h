#pragma once

#include <drone_mapper/drone/IDroneControl.h>
#include <drone_mapper/mocks/IDroneMovement.h>
#include <drone_mapper/mocks/IGPS.h>
#include <drone_mapper/mocks/ILidar.h>
#include <drone_mapper/map/IMap3D.h>
#include <drone_mapper/drone/IMappingAlgorithm.h>
#include <drone_mapper/mission/IMissionControl.h>
#include <drone_mapper/map/IMutableMap3D.h>
#include <drone_mapper/simulation/ISimulationRun.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

class SimulationRunImpl final : public ISimulationRun {
public:
    SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                      std::unique_ptr<IMutableMap3D> output_map,
                      std::unique_ptr<IGPS> gps,
                      std::unique_ptr<IDroneMovement> movement,
                      std::unique_ptr<ILidar> lidar,
                      std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<IDroneControl> drone_control,
                      std::unique_ptr<IMissionControl> mission_control,
                      // Changed: stores run metadata needed to build SimulationResult.
                      types::SimulationConfigData simulation_config,
                      types::MissionConfigData mission_config,
                      std::filesystem::path output_map_file);

    // Changed: matches ISimulationRun's new simulation-level result.
    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<const IMap3D> hidden_map_;
    std::unique_ptr<IMutableMap3D> output_map_;
    std::unique_ptr<IGPS> gps_;
    std::unique_ptr<IDroneMovement> movement_;
    std::unique_ptr<ILidar> lidar_;
    std::unique_ptr<IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<IDroneControl> drone_control_;
    std::unique_ptr<IMissionControl> mission_control_;
    // Changed: retained so run() can return the configs and output path in SimulationResult.
    types::SimulationConfigData simulation_config_;
    types::MissionConfigData mission_config_;
    std::filesystem::path output_map_file_;
};

} // namespace drone_mapper
