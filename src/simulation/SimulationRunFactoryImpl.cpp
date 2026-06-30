#include <drone_mapper/simulation/SimulationRunFactoryImpl.h>

#include <drone_mapper/drone/DroneControlImpl.h>
#include <drone_mapper/map/Map3DImpl.h>
#include <drone_mapper/drone/MappingAlgorithmImpl.h>
#include <drone_mapper/mission/MissionControlImpl.h>
#include <drone_mapper/mocks/MockGPS.h>
#include <drone_mapper/mocks/MockLidar.h>
#include <drone_mapper/mocks/MockMovement.h>
#include <drone_mapper/simulation/SimulationRunImpl.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace drone_mapper {
namespace {

// Loads a 3-D NPY file into a shared NpyArray. Throws std::runtime_error carrying the tinynpy error
// string when the file is missing, malformed, or not 3-D.
std::shared_ptr<NpyArray> loadNpyArray(const std::filesystem::path& path) {
    auto map = std::make_shared<NpyArray>();
    const std::string path_string = path.string();
    const char* error = map->LoadNPY(path_string.c_str());
    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY file: ") + error);
    }
    return map;
}

// True when the mission supplied real bounds (not the all-zero default produced when the mission YAML
// has no `boundaries` block). An all-zero MappingBounds means "no restriction — map the whole map".
bool missionBoundsSpecified(const types::MappingBounds& b) {
    const auto z_x = 0.0 * x_extent[cm];
    const auto z_y = 0.0 * y_extent[cm];
    const auto z_h = 0.0 * z_extent[cm];
    return !(b.min_x == z_x && b.max_x == z_x &&
             b.min_y == z_y && b.max_y == z_y &&
             b.min_height == z_h && b.max_height == z_h);
}

// Per-axis intersection of two boundary boxes.
types::MappingBounds intersectBounds(const types::MappingBounds& a, const types::MappingBounds& b) {
    return types::MappingBounds{
        std::max(a.min_x, b.min_x),           std::min(a.max_x, b.max_x),
        std::max(a.min_y, b.min_y),           std::min(a.max_y, b.max_y),
        std::max(a.min_height, b.min_height), std::min(a.max_height, b.max_height),
    };
}

} // namespace

types::MapConfig SimulationRunFactoryImpl::outputMapConfig(const types::MapConfig& hidden_config,
                                                          const types::MappingBounds& mission_bounds) {
    // No mission bounds → output map spans the full hidden extent (the prior behaviour, so a mission
    // whose bounds equal the map — e.g. the benchmark — is byte-for-byte identical).
    if (!missionBoundsSpecified(mission_bounds)) {
        return hidden_config;
    }
    // Narrow the output map to (hidden ∩ mission). The offset places array index 0 at the lower
    // corner of the intersection, so atVoxel/set/isInBounds (and therefore the planner and scans)
    // treat everything outside the bounded region as OutOfBounds.
    const types::MappingBounds isect = intersectBounds(hidden_config.boundaries, mission_bounds);
    types::MapConfig cfg = hidden_config;
    cfg.boundaries = isect;
    cfg.offset = Position3D{-isect.min_x, -isect.min_y, -isect.min_height};
    return cfg;
}

Position3D SimulationRunFactoryImpl::snapToCellCenter(const Position3D& position,
                                                     const types::MapConfig& config) {
    const double res = config.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        return position; // degenerate config: nothing to snap to
    }
    const double ox = config.offset.x.numerical_value_in(cm);
    const double oy = config.offset.y.numerical_value_in(cm);
    const double oz = config.offset.z.numerical_value_in(cm);
    // Cell index = floor((world + offset) / res); the cell centre is (index + 0.5) * res - offset.
    const auto centre = [res](double world, double off) {
        const double index = std::floor((world + off) / res);
        return (index + 0.5) * res - off;
    };
    return Position3D{
        centre(position.x.numerical_value_in(cm), ox) * x_extent[cm],
        centre(position.y.numerical_value_in(cm), oy) * y_extent[cm],
        centre(position.z.numerical_value_in(cm), oz) * z_extent[cm],
    };
}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // Hidden map: loaded from the NPY named in the simulation config. MappingBounds is left default
    // because Map3DImpl derives the real spatial extent from the NPY shape plus offset+resolution.
    const types::MapConfig hidden_map_config{
        types::MappingBounds{},
        simulation.map_offset,
        simulation.map_resolution,
    };
    auto hidden_map = std::make_unique<Map3DImpl>(
        loadNpyArray(simulation.map_filename), hidden_map_config);

    // Output map: an empty NpyArray sized from the *constructed* hidden map's resolved config (its
    // boundaries/offset come from the NPY shape, not the zeroed placeholder above). When the mission
    // specifies bounds, the output map is narrowed to (hidden ∩ mission) so the drone only maps the
    // bounded region; otherwise it spans the full hidden extent (benchmark behaviour preserved).
    const types::MapConfig hidden_resolved = hidden_map->getMapConfig();
    const types::MapConfig output_map_config =
        outputMapConfig(hidden_resolved, mission.mission_bounds);
    auto output_map = std::make_unique<Map3DImpl>(
        std::make_shared<NpyArray>(), output_map_config);

    // GPS starts at the configured position SNAPPED to its output-grid cell centre, so the drone
    // begins exactly on the planner grid (Checkpoint A); altitude is 0 (drone level). The 3-arg ctor
    // stores the GPS resolution from the mission config. (Start-position validity is still checked in
    // SimulationRunImpl::run() against the configured position, which is the same cell.)
    auto gps = std::make_unique<MockGPS>(
        snapToCellCenter(simulation.initial_drone_position, output_map_config),
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    // MockMovement validates moves against the hidden ground-truth map using the drone's radius, so
    // a drone that does not fit physically cannot move into the space (collision ownership lives here,
    // the low-churn HLD option, keeping the hidden map a live dependency).
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone.radius);
    // MockLidar ray-marches against the hidden map using the live GPS pose.
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    // MappingAlgorithmImpl plans off the output map; the base ctor stores mission/lidar/drone configs.
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(
        mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm);
    // DroneControlImpl reads the lidar config straight off the injected lidar (ILidar::config()),
    // so no separate config injection is needed.

    // Output file naming: output_results/<map_stem>_run<N>_output.npy. <map_stem> identifies the
    // simulated environment at a glance; <N> is a process-lifetime monotonic counter that keeps
    // names unique across the cartesian-product loop (atomic for any future parallel use).
    static std::atomic<std::size_t> s_run_counter{0};
    const auto run_idx = s_run_counter.fetch_add(1, std::memory_order_relaxed);

    const std::filesystem::path output_dir = output_path / "output_results";
    std::filesystem::create_directories(output_dir); // idempotent

    const std::string stem =
        simulation.map_filename.stem().string() + "_run" + std::to_string(run_idx) + "_output";
    const std::filesystem::path output_map_file = output_dir / (stem + ".npy");

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace drone_mapper
