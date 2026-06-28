#pragma once

#include <drone_mapper/drone/IDroneControl.h>
#include <drone_mapper/map/IMap3D.h>
#include <drone_mapper/mission/IMissionControl.h>
#include <drone_mapper/map/IMutableMap3D.h>

#include <filesystem>

namespace drone_mapper {

class MissionControlImpl final : public IMissionControl {
public:
    MissionControlImpl(types::MissionConfigData mission,
                       types::DroneConfigData drone,
                       const IMap3D& hidden_map,
                       IMutableMap3D& output_map,
                       IDroneControl& drone_control,
                       std::filesystem::path output_map_file);

    [[nodiscard]] types::MissionRunResult runMission() override;

private:
    types::MissionConfigData mission_;
    types::DroneConfigData drone_;
    const IMap3D& hidden_map_;
    IMutableMap3D& output_map_;
    IDroneControl& drone_control_;
    std::filesystem::path output_map_file_;
};

} // namespace drone_mapper
