#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>
#include <vector>

namespace drone_mapper {

// Source .yaml paths for a single run (one simulation × mission × drone × lidar combination). The
// frozen SimulationResult does not carry these, so the caller (main) supplies them index-aligned with
// SimulationManagerReport::runs to let the writer emit the exact spec output format.
struct RunPaths {
    std::filesystem::path simulation_config{};
    std::filesystem::path mission_config{};
    std::filesystem::path drone_config{};
    std::filesystem::path lidar_config{};
};

class SimulationReportWriter {
public:
    // Writes simulation_output.yaml to output_path in the spec `score_report` hierarchy.
    // composition_file: the input simulation_compositions.yaml path (written into the header).
    // run_paths: optional, index-aligned with report.runs. When supplied (and the size matches), the
    //   writer groups by simulation_config/mission_config path and emits drone_config/lidar_config per
    //   run — the exact spec format. When empty (e.g. the SimulationManager standalone write, which has
    //   no source paths), it falls back to grouping by map filename / mission identity.
    static void write(const types::SimulationManagerReport& report,
                      const std::filesystem::path& output_path,
                      const std::filesystem::path& composition_file,
                      const std::vector<RunPaths>& run_paths = {});
};

} // namespace drone_mapper
