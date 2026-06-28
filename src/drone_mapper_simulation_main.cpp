#include <drone_mapper/simulation/SimulationManager.h>
#include <drone_mapper/simulation/SimulationRunFactoryImpl.h>
#include <drone_mapper/utils/ErrorLogger.h>
#include <drone_mapper/utils/SimulationReportWriter.h>
#include <drone_mapper/utils/YamlConfigParser.h>

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

// drone_mapper_simulation
//
// Usage:
//   ./drone_mapper_simulation [<simulation_compositions.yaml>] [<output_path>]
//     no args → composition = "simulation.yaml" in cwd; output = cwd
//     1 arg   → composition = argv[1];                   output = cwd
//     2 args  → composition = argv[1];                   output = argv[2]
// Relative paths resolve against the cwd; absolute paths are used as-is.
//
// Outputs under output_path/:
//   simulation_output.yaml          — the score report for every run
//   output_results/                 — per-run output maps + error logs:
//     main_errors.log               — composition-parse / top-level errors
//     simulation_manager_errors.log — factory-level run failures
//     <map_stem>_run<N>_output.npy  — per-run output maps

int main(int argc, char** argv) {
    try {
        const std::filesystem::path composition_file =
            (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
        const std::filesystem::path output_path =
            (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

        std::filesystem::create_directories(output_path / "output_results");

        // Composition-parse / top-level errors. The file is created lazily on first log().
        drone_mapper::ErrorLogger logger{output_path / "output_results" / "main_errors.log"};

        // Parse the composition into one SimulationCompositionData; comp_paths carries the source
        // .yaml paths (index-aligned with the groups/drones/lidars) so the report can emit them.
        drone_mapper::config::CompositionPaths comp_paths;
        const auto composition =
            drone_mapper::config::parseSimulationCompositions(composition_file, logger, &comp_paths);

        if (composition.simulation_mission_groups.empty()) {
            std::cerr << "Error: no valid simulation configurations found in '"
                      << composition_file.string() << "'.\nCheck '"
                      << (output_path / "output_results" / "main_errors.log").string()
                      << "' for details.\n";
            return 1;
        }

        auto factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        drone_mapper::SimulationManager manager{std::move(factory)};
        const drone_mapper::types::SimulationManagerReport report = manager.run(composition, output_path);

        // Replay the manager's cartesian order (per group: mission × drone × lidar) to build RunPaths
        // aligned 1:1 with report.runs. This holds even on group failure — the manager still emits one
        // result per combination.
        std::vector<drone_mapper::RunPaths> run_paths;
        const bool paths_aligned =
            comp_paths.groups.size() == composition.simulation_mission_groups.size();
        if (paths_aligned) {
            for (std::size_t g = 0; g < composition.simulation_mission_groups.size(); ++g) {
                const auto& missions = std::get<1>(composition.simulation_mission_groups[g]);
                const auto& gp = comp_paths.groups[g];
                for (std::size_t m = 0; m < missions.size(); ++m) {
                    for (std::size_t d = 0; d < composition.drones.size(); ++d) {
                        for (std::size_t l = 0; l < composition.lidars.size(); ++l) {
                            run_paths.push_back(drone_mapper::RunPaths{
                                gp.simulation_config,
                                m < gp.mission_configs.size() ? gp.mission_configs[m]
                                                              : std::filesystem::path{},
                                d < comp_paths.drone_configs.size() ? comp_paths.drone_configs[d]
                                                                    : std::filesystem::path{},
                                l < comp_paths.lidar_configs.size() ? comp_paths.lidar_configs[l]
                                                                    : std::filesystem::path{},
                            });
                        }
                    }
                }
            }
        }

        // Authoritative path-aware write (overwrites the manager's standalone fallback). When the
        // counts don't line up, fall back to map-filename grouping by passing no paths.
        drone_mapper::SimulationReportWriter::write(
            report, output_path, composition_file,
            run_paths.size() == report.runs.size() ? run_paths
                                                    : std::vector<drone_mapper::RunPaths>{});

        const std::size_t n = report.runs.size();
        std::cout << "Simulation complete: ran " << n << " run" << (n == 1u ? "" : "s") << ".\n"
                  << "Output:  " << (output_path / "simulation_output.yaml").string() << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }
}
