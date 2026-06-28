#pragma once

#include <drone_mapper/drone/IDroneControl.h>
#include <drone_mapper/mocks/IDroneMovement.h>
#include <drone_mapper/mocks/IGPS.h>
#include <drone_mapper/mocks/ILidar.h>
#include <drone_mapper/drone/IMappingAlgorithm.h>
#include <drone_mapper/map/IMutableMap3D.h>

#include <optional>

namespace drone_mapper {

class DroneControlImpl final : public IDroneControl {
public:
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    // The previous step's scan, passed to the algorithm next call (nullptr on the first call).
    std::optional<types::LidarScanResult> latest_scan_result_{};
    std::size_t step_index_ = 0;
};

} // namespace drone_mapper
