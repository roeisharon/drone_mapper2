#pragma once

// Shared helpers for the integration-test translation units (real-algorithm and mock-algorithm).
// Included ONLY by files under tests/integration/. Every helper is `inline` so the TUs share a single
// definition without ODR issues, and a helper left unused in one TU does not trip -Wunused.

#include <drone_mapper/map/Map3DImpl.h>
#include <drone_mapper/utils/ErrorLogger.h>
#include <drone_mapper/utils/YamlConfigParser.h>
#include <drone_mapper/Types.h>
#include <drone_mapper/Units.h>

#include <TinyNPY.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace drone_mapper;
namespace fs = std::filesystem;

// Course inputs live at <repo>/inputs; the test binary is run from the repo root (like the existing
// data_maps/ tests), so a relative path resolves.
inline const fs::path kInputs{"inputs"};

// A fresh, empty temp output tree per test (with output_results/, which the factory/manager expect).
inline fs::path freshOutputDir(const std::string& tag) {
    const fs::path dir = fs::temp_directory_path() / ("dm_integration_" + tag);
    fs::remove_all(dir);
    fs::create_directories(dir / "output_results");
    return dir;
}

inline std::shared_ptr<NpyArray> loadNpy(const fs::path& path) {
    auto array = std::make_shared<NpyArray>();
    const char* err = array->LoadNPY(path.string().c_str());
    if (err != nullptr) {
        throw std::runtime_error(std::string("LoadNPY failed: ") + err);
    }
    return array;
}

inline Position3D worldCm(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

inline std::string readFile(const fs::path& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Occupancy census of a saved output map, obtained by RELOADING it (also exercises save/reload) and
// sampling every cell centre of its configured region.
struct MapStats {
    long mapped = 0;   // not Unmapped and not OutOfBounds — i.e. genuinely observed
    long empty = 0;
    long occupied = 0;
    long total = 0;
    int  nx = 0, ny = 0, nz = 0;
};

inline MapStats censusOutputMap(const fs::path& npy, const types::MapConfig& cfg) {
    MapStats s;
    Map3DImpl map{loadNpy(npy), cfg};
    const double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        return s;
    }
    const double x0 = cfg.boundaries.min_x.numerical_value_in(cm);
    const double y0 = cfg.boundaries.min_y.numerical_value_in(cm);
    const double z0 = cfg.boundaries.min_height.numerical_value_in(cm);
    s.nx = static_cast<int>(std::lround((cfg.boundaries.max_x.numerical_value_in(cm) - x0) / res));
    s.ny = static_cast<int>(std::lround((cfg.boundaries.max_y.numerical_value_in(cm) - y0) / res));
    s.nz = static_cast<int>(std::lround((cfg.boundaries.max_height.numerical_value_in(cm) - z0) / res));
    for (int ix = 0; ix < s.nx; ++ix) {
        for (int iy = 0; iy < s.ny; ++iy) {
            for (int iz = 0; iz < s.nz; ++iz) {
                const auto v = map.atVoxel(worldCm(x0 + (ix + 0.5) * res,
                                                   y0 + (iy + 0.5) * res,
                                                   z0 + (iz + 0.5) * res));
                ++s.total;
                if (v == types::VoxelOccupancy::Empty) ++s.empty;
                else if (v == types::VoxelOccupancy::Occupied) ++s.occupied;
                if (v != types::VoxelOccupancy::Unmapped && v != types::VoxelOccupancy::OutOfBounds) {
                    ++s.mapped;
                }
            }
        }
    }
    return s;
}

// Parses the small-room scenario (fast: ~200 steps for the small drone; the room fits it well).
struct SmallRoomInputs {
    types::SimulationConfigData simulation;
    types::MissionConfigData    mission;
    types::DroneConfigData      drone_small;
    types::DroneConfigData      drone_large;
    types::LidarConfigData      lidar_short;
    types::LidarConfigData      lidar_long;
};

inline SmallRoomInputs parseSmallRoom(ErrorLogger& logger) {
    SmallRoomInputs in;
    in.simulation  = config::parseSimulationConfig(kInputs / "simulation" / "small_simulation_room.yaml", logger);
    in.mission     = config::parseMissionConfig(kInputs / "mission" / "small_mission_room.yaml", logger);
    in.drone_small = config::parseDroneConfig(kInputs / "drone" / "drone_small.yaml", logger);
    in.drone_large = config::parseDroneConfig(kInputs / "drone" / "drone_large.yaml", logger);
    in.lidar_short = config::parseLidarConfig(kInputs / "lidar" / "lidar_short.yaml", logger);
    in.lidar_long  = config::parseLidarConfig(kInputs / "lidar" / "lidar_long.yaml", logger);
    return in;
}
