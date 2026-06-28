#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class ILidar {
public:
    virtual ~ILidar() = default;

    [[nodiscard]] virtual types::LidarScanResult scan(Orientation scan_orientation) const = 0;

    // Change: 20.6 - exposes the lidar's configuration so consumers (e.g. DroneControl) can read it.
    [[nodiscard]] virtual types::LidarConfigData config() const = 0;
};

} // namespace drone_mapper
