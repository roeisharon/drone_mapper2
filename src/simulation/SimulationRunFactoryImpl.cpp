#include <drone_mapper/simulation/SimulationRunFactoryImpl.h>

#include <drone_mapper/drone/DroneControlImpl.h>
#include <drone_mapper/map/Map3DImpl.h>
#include <drone_mapper/drone/MappingAlgorithmImpl.h>
#include <drone_mapper/mission/MissionControlImpl.h>
#include <drone_mapper/mocks/MockGPS.h>
#include <drone_mapper/mocks/MockLidar.h>
#include <drone_mapper/mocks/MockMovement.h>
#include <drone_mapper/simulation/SimulationRunImpl.h>

#include <atomic>
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

} // namespace

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
    // boundaries/offset come from the NPY shape, not the zeroed placeholder above). Using the
    // placeholder here would yield a 0x0x0 map and make save() throw. Base build keeps the same
    // resolution as the hidden map (output_mapping_resolution_factor handling is a bonus).
    const types::MapConfig hidden_resolved = hidden_map->getMapConfig();
    const types::MapConfig output_map_config{
        hidden_resolved.boundaries,
        hidden_resolved.offset,
        hidden_resolved.resolution,
    };
    auto output_map = std::make_unique<Map3DImpl>(
        std::make_shared<NpyArray>(), output_map_config);

    // GPS starts at the configured position and horizontal angle; altitude is 0 (drone level). The
    // 3-arg ctor stores the GPS resolution from the mission config.
    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
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
