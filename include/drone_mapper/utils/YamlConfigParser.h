#pragma once

#include <drone_mapper/Types.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/utils/ErrorLogger.h>

#include <filesystem>
#include <optional>
#include <vector>

namespace drone_mapper::config {

// Side-band carriers for source .yaml paths. The frozen skeleton data types
// (SimulationCompositionData, MissionConfigData, …) do not record which file each sub-config came
// from, but the report writer must emit those paths in simulation_output.yaml. The parser hands
// them back through these structs. None duplicate a skeleton type — they hold only what the frozen
// types cannot. (Mission boundaries are no longer carried here: since 20.6 they live on
// MissionConfigData.mission_bounds.)

// Source .yaml paths for one simulation→missions group, aligned with one entry of
// SimulationCompositionData.simulation_mission_groups.
struct CompositionGroupPaths {
    std::filesystem::path              simulation_config;  // the group's simulation_config file
    std::vector<std::filesystem::path> mission_configs;    // aligned with the group's mission list
};

// Source .yaml paths for a whole composition, index-aligned with the returned
// SimulationCompositionData (groups ↔ simulation_mission_groups, drones ↔ drones, lidars ↔ lidars).
struct CompositionPaths {
    std::vector<CompositionGroupPaths> groups;
    std::vector<std::filesystem::path> drone_configs;
    std::vector<std::filesystem::path> lidar_configs;
};

// BONUS: one side of a comparison_config (original or target).
struct MapComparisonSpec {
    PhysicalLength                      map_resolution{};
    Position3D                          map_offset{};
    std::optional<types::MappingBounds> map_boundaries{};  // optional sub-region
};

// BONUS: parsed comparison_config file (maps_comparison utility).
struct ComparisonConfig {
    MapComparisonSpec original;
    MapComparisonSpec target;
};

// Each parser loads one YAML file, populates the matching data struct, and logs (but does NOT throw)
// for missing required fields, returning a fully populated struct even on partial failure. Each
// file's body may be wrapped under a top-level key named after the config kind (drone_config,
// lidar_config, mission_config, simulation_config); a flat document (fields at the root) is also
// accepted.

// Parses a drone_config YAML file into DroneConfigData.
// Required fields: dimensions_cm (→ radius), max_rotate_deg, max_advance_cm, max_elevate_cm.
types::DroneConfigData parseDroneConfig(const std::filesystem::path& path, ErrorLogger& logger);

// Parses a lidar_config YAML file into LidarConfigData.
// Required fields: z_min_cm, z_max_cm, d_cm, fov_circles.
types::LidarConfigData parseLidarConfig(const std::filesystem::path& path, ErrorLogger& logger);

// Parses a mission_config YAML file into MissionConfigData.
// Required: max_steps, gps_resolution_cm. Optional: output_mapping_resolution_factor (spec: integer,
// default 1 when absent). The spec's `boundaries` block (x/y/height_boundary × min_cm/max_cm) is read
// into MissionConfigData.mission_bounds; when the block is absent, mission_bounds stays default.
types::MissionConfigData parseMissionConfig(const std::filesystem::path& path, ErrorLogger& logger);

// Parses a simulation_config YAML file into SimulationConfigData.
// Required: map_filename (resolved relative to the config file's directory), map_resolution_cm.
// Optional: map_axes_offset (x_offset/y_offset/height_offset, default 0), initial_drone_position
// (x_cm/y_cm/height_cm, default 0), initial_angle_deg (default 0).
types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path,
                                                  ErrorLogger& logger);

// BONUS: parses a comparison_config YAML file (maps_comparison utility). Returns nullopt (and logs)
// if the file cannot be read or has neither an `original` nor a `target` block.
std::optional<ComparisonConfig> parseComparisonConfig(const std::filesystem::path& path,
                                                      ErrorLogger& logger);

// Parses a simulation_compositions YAML file into a single SimulationCompositionData. The spec nests
// each simulation with its own mission list under `simulations: - simulation_config / mission_configs`,
// plus global `drone_configs` and `lidar_configs` shared across every simulation; these populate
// simulation_mission_groups, drones, and lidars respectively. The body may be wrapped under a
// top-level `simulation_compositions:` key, or be flat. All sub-config paths resolve relative to the
// composition file's directory. A simulation entry with no readable simulation_config is skipped with
// an error logged. Returns an empty composition (no groups) when the file itself cannot be read.
// When out_paths != nullptr it is filled with the index-aligned source .yaml paths.
types::SimulationCompositionData
parseSimulationCompositions(const std::filesystem::path& path,
                            ErrorLogger& logger,
                            CompositionPaths* out_paths = nullptr);

} // namespace drone_mapper::config
