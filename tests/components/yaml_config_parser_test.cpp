#include <gtest/gtest.h>

#include <drone_mapper/utils/YamlConfigParser.h>
#include <drone_mapper/utils/ErrorLogger.h>
#include <drone_mapper/Units.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <tuple>

using drone_mapper::cm;
using drone_mapper::deg;
using drone_mapper::ErrorLogger;
using namespace drone_mapper::config;

// Fixture (suite name "YamlConfigParser") writes temp YAML files and inspects an ErrorLogger.
class YamlConfigParser : public ::testing::Test {
protected:
    std::filesystem::path tmp_dir_;

    void SetUp() override {
        tmp_dir_ = std::filesystem::temp_directory_path() / "yaml_config_parser_test";
        std::filesystem::remove_all(tmp_dir_);
        std::filesystem::create_directories(tmp_dir_);
    }
    void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

    // Writes content to a file inside tmp_dir_ (creating subdirs), returns its path.
    std::filesystem::path write(const std::string& name, const std::string& content) {
        const auto path = tmp_dir_ / name;
        std::filesystem::create_directories(path.parent_path());
        std::ofstream{path} << content;
        return path;
    }

    std::filesystem::path log_path() const { return tmp_dir_ / "errors.log"; }
    ErrorLogger makeLogger() { return ErrorLogger{log_path()}; }
    bool logExists() const { return std::filesystem::exists(log_path()); }

    // Writes a complete one-of-each composition tree and returns the composition file path.
    std::filesystem::path writeBasicComposition() {
        write("drone_configs/d.yaml",
              "dimensions_cm: 30\nmax_rotate_deg: 45\nmax_advance_cm: 50\nmax_elevate_cm: 40\n");
        write("lidar_configs/l.yaml", "z_min_cm: 20\nz_max_cm: 120\nd_cm: 2.5\nfov_circles: 5\n");
        write("missions/m.yaml", "max_steps: 100\ngps_resolution_cm: 10\n");
        write("sims/s.yaml", "map_filename: \"x.npy\"\nmap_resolution_cm: 10\n");
        return write("comp.yaml",
                     "simulation_compositions:\n"
                     "  simulations:\n"
                     "    - simulation_config: \"sims/s.yaml\"\n"
                     "      mission_configs:\n        - \"missions/m.yaml\"\n"
                     "  drone_configs:\n    - \"drone_configs/d.yaml\"\n"
                     "  lidar_configs:\n    - \"lidar_configs/l.yaml\"\n");
    }
};

// parseDroneConfig

// A fully specified flat drone config populates every field with the exact (unit-converted) value.
// `dimensions_cm` is the drone's sphere DIAMETER (per the config spec); radius = diameter / 2.
TEST_F(YamlConfigParser, DroneConfigAllFieldsParsed) {
    const auto p = write("drone.yaml",
                         "dimensions_cm: 30\nmax_rotate_deg: 45\nmax_advance_cm: 50\nmax_elevate_cm: 40\n");
    auto logger = makeLogger();
    const auto cfg = parseDroneConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.radius.numerical_value_in(cm), 15.0); // 30 cm diameter → 15 cm radius
    EXPECT_DOUBLE_EQ(cfg.max_rotate.numerical_value_in(deg), 45.0);
    EXPECT_DOUBLE_EQ(cfg.max_advance.numerical_value_in(cm), 50.0);
    EXPECT_DOUBLE_EQ(cfg.max_elevate.numerical_value_in(cm), 40.0);
    EXPECT_FALSE(logExists());
}

// The body may be wrapped under a top-level `drone_config:` key.
TEST_F(YamlConfigParser, DroneConfigWrappedFormParsed) {
    const auto p = write("drone.yaml",
                         "drone_config:\n  dimensions_cm: 12\n  max_rotate_deg: 10\n"
                         "  max_advance_cm: 20\n  max_elevate_cm: 5\n");
    auto logger = makeLogger();
    const auto cfg = parseDroneConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.radius.numerical_value_in(cm), 6.0); // 12 cm diameter → 6 cm radius
    EXPECT_DOUBLE_EQ(cfg.max_elevate.numerical_value_in(cm), 5.0);
}

// A missing required field defaults to 0 and writes an error-log entry.
TEST_F(YamlConfigParser, DroneConfigMissingFieldLogsAndDefaults) {
    const auto p = write("drone.yaml", "dimensions_cm: 30\n"); // others missing
    auto logger = makeLogger();
    const auto cfg = parseDroneConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.radius.numerical_value_in(cm), 15.0); // 30 cm diameter → 15 cm radius
    EXPECT_DOUBLE_EQ(cfg.max_advance.numerical_value_in(cm), 0.0);
    EXPECT_TRUE(logExists());
}

// parseLidarConfig

// A fully specified lidar config populates every field; fov_circles is an integer count.
TEST_F(YamlConfigParser, LidarConfigAllFieldsParsed) {
    const auto p = write("lidar.yaml", "z_min_cm: 20\nz_max_cm: 120\nd_cm: 2.5\nfov_circles: 5\n");
    auto logger = makeLogger();
    const auto cfg = parseLidarConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.z_min.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(cfg.z_max.numerical_value_in(cm), 120.0);
    EXPECT_DOUBLE_EQ(cfg.d.numerical_value_in(cm), 2.5);
    EXPECT_EQ(cfg.fov_circles, 5u);
    EXPECT_FALSE(logExists());
}

// A missing lidar field logs and defaults.
TEST_F(YamlConfigParser, LidarConfigMissingFieldLogs) {
    const auto p = write("lidar.yaml", "z_min_cm: 20\nz_max_cm: 120\n"); // d_cm, fov_circles missing
    auto logger = makeLogger();
    const auto cfg = parseLidarConfig(p, logger);
    EXPECT_EQ(cfg.fov_circles, 0u);
    EXPECT_TRUE(logExists());
}

// parseMissionConfig

// max_steps and gps_resolution are read; an omitted factor defaults to 1.
TEST_F(YamlConfigParser, MissionConfigBasicFieldsAndDefaultFactor) {
    const auto p = write("mission.yaml", "max_steps: 2400\ngps_resolution_cm: 10\n");
    auto logger = makeLogger();
    const auto cfg = parseMissionConfig(p, logger);
    EXPECT_EQ(cfg.max_steps, 2400u);
    EXPECT_DOUBLE_EQ(cfg.gps_resolution.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(cfg.output_mapping_resolution_factor, 1.0);
}

// A present factor (including < 1) is stored verbatim; clamping is the run's concern, not the parser's.
TEST_F(YamlConfigParser, MissionConfigFactorStoredVerbatim) {
    const auto p = write("mission.yaml",
                         "max_steps: 10\ngps_resolution_cm: 10\noutput_mapping_resolution_factor: 2\n");
    auto logger = makeLogger();
    EXPECT_DOUBLE_EQ(parseMissionConfig(p, logger).output_mapping_resolution_factor, 2.0);

    const auto p2 = write("mission2.yaml",
                          "max_steps: 10\ngps_resolution_cm: 10\noutput_mapping_resolution_factor: 0.5\n");
    auto logger2 = makeLogger();
    EXPECT_DOUBLE_EQ(parseMissionConfig(p2, logger2).output_mapping_resolution_factor, 0.5);
}

// The 20.6 `boundaries` block is parsed into MissionConfigData.mission_bounds (all three axes).
TEST_F(YamlConfigParser, MissionConfigBoundariesParsedIntoMissionBounds) {
    const auto p = write("mission.yaml",
                         "max_steps: 10\ngps_resolution_cm: 10\n"
                         "boundaries:\n"
                         "  x_boundary:\n    min_cm: -500\n    max_cm: -30\n"
                         "  y_boundary:\n    min_cm: 30\n    max_cm: 400\n"
                         "  height_boundary:\n    min_cm: -30\n    max_cm: 300\n");
    auto logger = makeLogger();
    const auto b = parseMissionConfig(p, logger).mission_bounds;
    EXPECT_DOUBLE_EQ(b.min_x.numerical_value_in(cm), -500.0);
    EXPECT_DOUBLE_EQ(b.max_x.numerical_value_in(cm), -30.0);
    EXPECT_DOUBLE_EQ(b.min_y.numerical_value_in(cm), 30.0);
    EXPECT_DOUBLE_EQ(b.max_y.numerical_value_in(cm), 400.0);
    EXPECT_DOUBLE_EQ(b.min_height.numerical_value_in(cm), -30.0);
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), 300.0);
}

// When the `boundaries` block is absent, mission_bounds stays default (all zero).
TEST_F(YamlConfigParser, MissionConfigBoundariesAbsentLeavesDefault) {
    const auto p = write("mission.yaml", "max_steps: 10\ngps_resolution_cm: 10\n");
    auto logger = makeLogger();
    const auto b = parseMissionConfig(p, logger).mission_bounds;
    EXPECT_DOUBLE_EQ(b.min_x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(b.max_height.numerical_value_in(cm), 0.0);
}

// A missing required mission field logs.
TEST_F(YamlConfigParser, MissionConfigMissingRequiredLogs) {
    const auto p = write("mission.yaml", "gps_resolution_cm: 10\n"); // max_steps missing
    auto logger = makeLogger();
    EXPECT_EQ(parseMissionConfig(p, logger).max_steps, 0u);
    EXPECT_TRUE(logExists());
}

// parseSimulationConfig

// All simulation fields parse; map_filename resolves relative to the config file's directory.
TEST_F(YamlConfigParser, SimulationConfigAllFieldsParsed) {
    const auto p = write("sims/sim.yaml",
                         "map_filename: \"office.npy\"\nmap_resolution_cm: 10\n"
                         "initial_drone_position:\n  x_cm: 250\n  y_cm: 200\n  height_cm: 150\n"
                         "initial_angle_deg: 90\n");
    auto logger = makeLogger();
    const auto cfg = parseSimulationConfig(p, logger);
    EXPECT_EQ(cfg.map_filename, tmp_dir_ / "sims" / "office.npy");
    EXPECT_DOUBLE_EQ(cfg.map_resolution.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(cfg.initial_drone_position.x.numerical_value_in(cm), 250.0);
    EXPECT_DOUBLE_EQ(cfg.initial_drone_position.y.numerical_value_in(cm), 200.0);
    EXPECT_DOUBLE_EQ(cfg.initial_drone_position.z.numerical_value_in(cm), 150.0);
    EXPECT_DOUBLE_EQ(cfg.initial_angle.numerical_value_in(deg), 90.0);
}

// map_axes_offset at the document root is parsed (positive values).
TEST_F(YamlConfigParser, SimulationConfigOffsetAtRoot) {
    const auto p = write("sim.yaml",
                         "map_filename: \"m.npy\"\nmap_resolution_cm: 10\n"
                         "map_axes_offset:\n  x_offset: 1000\n  y_offset: 1000\n  height_offset: 1500\n");
    auto logger = makeLogger();
    const auto cfg = parseSimulationConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.map_offset.x.numerical_value_in(cm), 1000.0);
    EXPECT_DOUBLE_EQ(cfg.map_offset.y.numerical_value_in(cm), 1000.0);
    EXPECT_DOUBLE_EQ(cfg.map_offset.z.numerical_value_in(cm), 1500.0);
}

// Negative offsets are accepted (the logical origin lies before the array start).
TEST_F(YamlConfigParser, SimulationConfigNegativeOffset) {
    const auto p = write("sim.yaml",
                         "map_filename: \"m.npy\"\nmap_resolution_cm: 10\n"
                         "map_axes_offset:\n  x_offset: -200\n  y_offset: -50\n  height_offset: -10\n");
    auto logger = makeLogger();
    const auto cfg = parseSimulationConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.map_offset.x.numerical_value_in(cm), -200.0);
    EXPECT_DOUBLE_EQ(cfg.map_offset.z.numerical_value_in(cm), -10.0);
}

// The benchmark-style key `map_axes_offset_cm` must be honoured too — otherwise a non-zero offset
// in those configs is silently dropped and the map is treated as starting at world 0.
TEST_F(YamlConfigParser, SimulationConfigOffsetCmKeyAccepted) {
    const auto p = write("sim.yaml",
                         "map_filename: \"m.npy\"\nmap_resolution_cm: 10\n"
                         "map_axes_offset_cm:\n  x_offset: -20\n  y_offset: -20\n  height_offset: -20\n");
    auto logger = makeLogger();
    const auto cfg = parseSimulationConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.map_offset.x.numerical_value_in(cm), -20.0);
    EXPECT_DOUBLE_EQ(cfg.map_offset.y.numerical_value_in(cm), -20.0);
    EXPECT_DOUBLE_EQ(cfg.map_offset.z.numerical_value_in(cm), -20.0);
}

// An absent offset defaults to (0,0,0).
TEST_F(YamlConfigParser, SimulationConfigOffsetAbsentDefaultsZero) {
    const auto p = write("sim.yaml", "map_filename: \"m.npy\"\nmap_resolution_cm: 10\n");
    auto logger = makeLogger();
    const auto cfg = parseSimulationConfig(p, logger);
    EXPECT_DOUBLE_EQ(cfg.map_offset.x.numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(cfg.map_offset.z.numerical_value_in(cm), 0.0);
}

// `z_cm` is accepted as a fallback alias for the vertical position key.
TEST_F(YamlConfigParser, SimulationConfigZCmFallbackForHeight) {
    const auto p = write("sim.yaml",
                         "map_filename: \"m.npy\"\nmap_resolution_cm: 10\n"
                         "initial_drone_position:\n  x_cm: 1\n  y_cm: 2\n  z_cm: 33\n");
    auto logger = makeLogger();
    EXPECT_DOUBLE_EQ(parseSimulationConfig(p, logger).initial_drone_position.z.numerical_value_in(cm), 33.0);
}

// When both height_cm and z_cm are present, height_cm wins.
TEST_F(YamlConfigParser, SimulationConfigHeightCmTakesPrecedenceOverZCm) {
    const auto p = write("sim.yaml",
                         "map_filename: \"m.npy\"\nmap_resolution_cm: 10\n"
                         "initial_drone_position:\n  height_cm: 7\n  z_cm: 99\n");
    auto logger = makeLogger();
    EXPECT_DOUBLE_EQ(parseSimulationConfig(p, logger).initial_drone_position.z.numerical_value_in(cm), 7.0);
}

// A missing map_filename logs (and leaves the path empty).
TEST_F(YamlConfigParser, SimulationConfigMissingMapFilenameLogs) {
    const auto p = write("sim.yaml", "map_resolution_cm: 10\n");
    auto logger = makeLogger();
    EXPECT_TRUE(parseSimulationConfig(p, logger).map_filename.empty());
    EXPECT_TRUE(logExists());
}

// parseSimulationCompositions

// A single-group composition parses into one group with one mission, plus the shared drone/lidar.
TEST_F(YamlConfigParser, CompositionSingleGroupParsed) {
    const auto p = writeBasicComposition();
    auto logger = makeLogger();
    const auto comp = parseSimulationCompositions(p, logger);
    ASSERT_EQ(comp.simulation_mission_groups.size(), 1u);
    const auto& [sim, missions] = comp.simulation_mission_groups[0];
    EXPECT_EQ(sim.map_filename, tmp_dir_ / "sims" / "x.npy");
    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0].max_steps, 100u);
    EXPECT_EQ(comp.drones.size(), 1u);
    EXPECT_EQ(comp.lidars.size(), 1u);
    EXPECT_EQ(comp.composition_file, p);
}

// Two simulation entries each become their own group, each carrying its own mission list.
TEST_F(YamlConfigParser, CompositionTwoGroupsEachWithOwnMissions) {
    write("sims/a.yaml", "map_filename: \"a.npy\"\nmap_resolution_cm: 10\n");
    write("sims/b.yaml", "map_filename: \"b.npy\"\nmap_resolution_cm: 10\n");
    write("missions/m1.yaml", "max_steps: 11\ngps_resolution_cm: 10\n");
    write("missions/m2.yaml", "max_steps: 22\ngps_resolution_cm: 10\n");
    write("missions/m3.yaml", "max_steps: 33\ngps_resolution_cm: 10\n");
    write("drone_configs/d.yaml",
          "dimensions_cm: 1\nmax_rotate_deg: 1\nmax_advance_cm: 1\nmax_elevate_cm: 1\n");
    write("lidar_configs/l.yaml", "z_min_cm: 1\nz_max_cm: 2\nd_cm: 1\nfov_circles: 1\n");
    const auto p = write("comp.yaml",
                         "simulation_compositions:\n"
                         "  simulations:\n"
                         "    - simulation_config: \"sims/a.yaml\"\n"
                         "      mission_configs:\n        - \"missions/m1.yaml\"\n        - \"missions/m2.yaml\"\n"
                         "    - simulation_config: \"sims/b.yaml\"\n"
                         "      mission_configs:\n        - \"missions/m3.yaml\"\n"
                         "  drone_configs:\n    - \"drone_configs/d.yaml\"\n"
                         "  lidar_configs:\n    - \"lidar_configs/l.yaml\"\n");
    auto logger = makeLogger();
    const auto comp = parseSimulationCompositions(p, logger);
    ASSERT_EQ(comp.simulation_mission_groups.size(), 2u);
    EXPECT_EQ(std::get<1>(comp.simulation_mission_groups[0]).size(), 2u);
    EXPECT_EQ(std::get<1>(comp.simulation_mission_groups[1]).size(), 1u);
    EXPECT_EQ(std::get<1>(comp.simulation_mission_groups[0])[1].max_steps, 22u);
    EXPECT_EQ(std::get<1>(comp.simulation_mission_groups[1])[0].max_steps, 33u);
}

// Drone/lidar lists are global — shared across all groups (parsed once).
TEST_F(YamlConfigParser, CompositionSharesGlobalDronesAndLidars) {
    write("sims/a.yaml", "map_filename: \"a.npy\"\nmap_resolution_cm: 10\n");
    write("sims/b.yaml", "map_filename: \"b.npy\"\nmap_resolution_cm: 10\n");
    write("missions/m.yaml", "max_steps: 1\ngps_resolution_cm: 10\n");
    write("drone_configs/d1.yaml",
          "dimensions_cm: 1\nmax_rotate_deg: 1\nmax_advance_cm: 1\nmax_elevate_cm: 1\n");
    write("drone_configs/d2.yaml",
          "dimensions_cm: 2\nmax_rotate_deg: 2\nmax_advance_cm: 2\nmax_elevate_cm: 2\n");
    write("lidar_configs/l.yaml", "z_min_cm: 1\nz_max_cm: 2\nd_cm: 1\nfov_circles: 1\n");
    const auto p = write("comp.yaml",
                         "simulation_compositions:\n"
                         "  simulations:\n"
                         "    - simulation_config: \"sims/a.yaml\"\n      mission_configs:\n        - \"missions/m.yaml\"\n"
                         "    - simulation_config: \"sims/b.yaml\"\n      mission_configs:\n        - \"missions/m.yaml\"\n"
                         "  drone_configs:\n    - \"drone_configs/d1.yaml\"\n    - \"drone_configs/d2.yaml\"\n"
                         "  lidar_configs:\n    - \"lidar_configs/l.yaml\"\n");
    auto logger = makeLogger();
    const auto comp = parseSimulationCompositions(p, logger);
    EXPECT_EQ(comp.simulation_mission_groups.size(), 2u);
    EXPECT_EQ(comp.drones.size(), 2u);
    EXPECT_EQ(comp.lidars.size(), 1u);
}

// A flat (un-wrapped) composition document is accepted.
TEST_F(YamlConfigParser, CompositionFlatFormAccepted) {
    write("sims/s.yaml", "map_filename: \"x.npy\"\nmap_resolution_cm: 10\n");
    write("missions/m.yaml", "max_steps: 5\ngps_resolution_cm: 10\n");
    write("drone_configs/d.yaml",
          "dimensions_cm: 1\nmax_rotate_deg: 1\nmax_advance_cm: 1\nmax_elevate_cm: 1\n");
    write("lidar_configs/l.yaml", "z_min_cm: 1\nz_max_cm: 2\nd_cm: 1\nfov_circles: 1\n");
    const auto p = write("comp.yaml",
                         "simulations:\n"
                         "  - simulation_config: \"sims/s.yaml\"\n    mission_configs:\n      - \"missions/m.yaml\"\n"
                         "drone_configs:\n  - \"drone_configs/d.yaml\"\n"
                         "lidar_configs:\n  - \"lidar_configs/l.yaml\"\n");
    auto logger = makeLogger();
    EXPECT_EQ(parseSimulationCompositions(p, logger).simulation_mission_groups.size(), 1u);
}

// A non-existent composition file returns an empty composition (no groups) and logs.
TEST_F(YamlConfigParser, CompositionMissingFileReturnsEmptyAndLogs) {
    auto logger = makeLogger();
    const auto comp = parseSimulationCompositions(tmp_dir_ / "nope.yaml", logger);
    EXPECT_TRUE(comp.simulation_mission_groups.empty());
    EXPECT_TRUE(logExists());
}

// A simulation entry missing its simulation_config key is skipped (others still parse) and logs.
TEST_F(YamlConfigParser, CompositionEntryMissingSimConfigIsSkipped) {
    write("sims/s.yaml", "map_filename: \"x.npy\"\nmap_resolution_cm: 10\n");
    write("missions/m.yaml", "max_steps: 5\ngps_resolution_cm: 10\n");
    write("drone_configs/d.yaml",
          "dimensions_cm: 1\nmax_rotate_deg: 1\nmax_advance_cm: 1\nmax_elevate_cm: 1\n");
    write("lidar_configs/l.yaml", "z_min_cm: 1\nz_max_cm: 2\nd_cm: 1\nfov_circles: 1\n");
    const auto p = write("comp.yaml",
                         "simulation_compositions:\n"
                         "  simulations:\n"
                         "    - mission_configs:\n        - \"missions/m.yaml\"\n" // no simulation_config
                         "    - simulation_config: \"sims/s.yaml\"\n      mission_configs:\n        - \"missions/m.yaml\"\n"
                         "  drone_configs:\n    - \"drone_configs/d.yaml\"\n"
                         "  lidar_configs:\n    - \"lidar_configs/l.yaml\"\n");
    auto logger = makeLogger();
    const auto comp = parseSimulationCompositions(p, logger);
    EXPECT_EQ(comp.simulation_mission_groups.size(), 1u); // only the valid entry survives
    EXPECT_TRUE(logExists());
}

// out_paths is filled index-aligned with the returned composition (groups, drones, lidars).
TEST_F(YamlConfigParser, CompositionOutPathsAreAligned) {
    const auto p = writeBasicComposition();
    auto logger = makeLogger();
    CompositionPaths paths;
    const auto comp = parseSimulationCompositions(p, logger, &paths);
    ASSERT_EQ(paths.groups.size(), comp.simulation_mission_groups.size());
    EXPECT_EQ(paths.groups[0].simulation_config, tmp_dir_ / "sims" / "s.yaml");
    ASSERT_EQ(paths.groups[0].mission_configs.size(), 1u);
    EXPECT_EQ(paths.groups[0].mission_configs[0], tmp_dir_ / "missions" / "m.yaml");
    ASSERT_EQ(paths.drone_configs.size(), 1u);
    EXPECT_EQ(paths.drone_configs[0], tmp_dir_ / "drone_configs" / "d.yaml");
    ASSERT_EQ(paths.lidar_configs.size(), 1u);
    EXPECT_EQ(paths.lidar_configs[0], tmp_dir_ / "lidar_configs" / "l.yaml");
}

// parseComparisonConfig (BONUS)

// Both sides parse their resolution, offset, and boundary blocks.
TEST_F(YamlConfigParser, ComparisonConfigBothSidesParsed) {
    const auto p = write("cmp.yaml",
                         "comparison_config:\n"
                         "  original:\n    map_res_cm: 10\n"
                         "    map_offset:\n      x_offset: 1000\n      y_offset: 1320\n      height_offset: 3200\n"
                         "    map_boundaries:\n      x_boundary:\n        min_cm: -500\n        max_cm: -30\n"
                         "  target:\n    map_res_cm: 5\n"
                         "    map_offset:\n      x_offset: 100\n      y_offset: 132\n      height_offset: 32\n");
    auto logger = makeLogger();
    const auto cfg = parseComparisonConfig(p, logger);
    ASSERT_TRUE(cfg.has_value());
    EXPECT_DOUBLE_EQ(cfg->original.map_resolution.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(cfg->original.map_offset.x.numerical_value_in(cm), 1000.0);
    ASSERT_TRUE(cfg->original.map_boundaries.has_value());
    EXPECT_DOUBLE_EQ(cfg->original.map_boundaries->min_x.numerical_value_in(cm), -500.0);
    EXPECT_DOUBLE_EQ(cfg->target.map_resolution.numerical_value_in(cm), 5.0);
    EXPECT_DOUBLE_EQ(cfg->target.map_offset.z.numerical_value_in(cm), 32.0);
}

// An unwrapped comparison document (no `comparison_config:` key) is accepted.
TEST_F(YamlConfigParser, ComparisonConfigUnwrappedAccepted) {
    const auto p = write("cmp.yaml", "original:\n  map_res_cm: 10\ntarget:\n  map_res_cm: 10\n");
    auto logger = makeLogger();
    EXPECT_TRUE(parseComparisonConfig(p, logger).has_value());
}

// A document with neither original nor target returns nullopt and logs.
TEST_F(YamlConfigParser, ComparisonConfigMissingBothSidesReturnsNullopt) {
    const auto p = write("cmp.yaml", "something_else: 1\n");
    auto logger = makeLogger();
    EXPECT_FALSE(parseComparisonConfig(p, logger).has_value());
    EXPECT_TRUE(logExists());
}

// A non-existent comparison file returns nullopt.
TEST_F(YamlConfigParser, ComparisonConfigMissingFileReturnsNullopt) {
    auto logger = makeLogger();
    EXPECT_FALSE(parseComparisonConfig(tmp_dir_ / "nope.yaml", logger).has_value());
}
