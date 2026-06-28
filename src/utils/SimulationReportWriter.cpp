#include <drone_mapper/utils/SimulationReportWriter.h>
#include <drone_mapper/Units.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace drone_mapper {
namespace {

// Resolution-request enum → YAML string.
std::string statusStr(types::ResolutionRequestStatus s) {
    switch (s) {
        case types::ResolutionRequestStatus::Accepted:        return "Accepted";
        case types::ResolutionRequestStatus::Ignored:         return "Ignored";
        case types::ResolutionRequestStatus::IgnoredTooSmall: return "IgnoredTooSmall";
    }
    return "Unknown";
}

// Mission status → spec's lowercase YAML string.
std::string missionStatusStr(types::MissionRunStatus s) {
    switch (s) {
        case types::MissionRunStatus::Completed: return "completed";
        case types::MissionRunStatus::MaxSteps:  return "max_steps";
        case types::MissionRunStatus::Error:     return "error";
    }
    return "error";
}

// Pairs one run result with its source .yaml paths (null when unknown — e.g. the SimulationManager
// standalone write, which has no access to the file paths).
struct RunEntry {
    const types::SimulationResult* result = nullptr;
    const RunPaths*                paths  = nullptr;
};

// Grouping keys. With paths: identify simulations/missions by their config-file path (the spec
// identity). Without paths: fall back to map filename / (max_steps, gps_resolution).
std::string simKey(const RunEntry& e) {
    if (e.paths != nullptr) return e.paths->simulation_config.string();
    return e.result->simulation_config.map_filename.string();
}

std::string missionKey(const RunEntry& e) {
    if (e.paths != nullptr) return e.paths->mission_config.string();
    return std::to_string(e.result->mission_config.max_steps) + "|" +
           std::to_string(e.result->mission_config.gps_resolution.numerical_value_in(cm));
}

struct Summary {
    int    total_runs  = 0;
    int    scored_runs = 0;
    int    error_runs  = 0;
    double avg_score   = 0.0;
    double min_score   = 0.0;
    double max_score   = 0.0;
};

// A run is "scored" when mission_score >= 0; a negative score means a factory/mission error.
Summary computeSummary(const std::vector<types::SimulationResult>& runs) {
    Summary s;
    s.total_runs = static_cast<int>(runs.size());

    double sum   = 0.0;
    double min_v = std::numeric_limits<double>::max();
    double max_v = std::numeric_limits<double>::lowest();

    for (const auto& r : runs) {
        if (r.mission_score >= 0.0) {
            ++s.scored_runs;
            sum  += r.mission_score;
            min_v = std::min(min_v, r.mission_score);
            max_v = std::max(max_v, r.mission_score);
        } else {
            ++s.error_runs;
        }
    }

    if (s.scored_runs > 0) {
        s.avg_score = sum / s.scored_runs;
        s.min_score = min_v;
        s.max_score = max_v;
    }
    return s;
}

// Emits one run entry (one drone × lidar combination).
// Spec field order: drone_config, lidar_config, status, steps, score, [error_ref].
void emitRun(YAML::Emitter& out, const RunEntry& e) {
    const auto& r = *e.result;
    out << YAML::BeginMap;

    out << YAML::Key << "drone_config" << YAML::Value
        << (e.paths != nullptr ? e.paths->drone_config.string() : std::string{});
    out << YAML::Key << "lidar_config" << YAML::Value
        << (e.paths != nullptr ? e.paths->lidar_config.string() : std::string{});

    if (!r.mission_results.empty()) {
        const auto& mr = r.mission_results.front();
        out << YAML::Key << "status" << YAML::Value << missionStatusStr(mr.status);
        out << YAML::Key << "steps"  << YAML::Value << static_cast<int>(mr.steps);
        out << YAML::Key << "score"  << YAML::Value << r.mission_score;
        // error_ref — spec uses a singular block with just a code; emit the first (most significant).
        if (!mr.errors.empty()) {
            out << YAML::Key << "error_ref" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "code" << YAML::Value << mr.errors.front().code;
            out << YAML::EndMap;
        }
    } else {
        // Factory-level / exception failure — no mission ran.
        out << YAML::Key << "status" << YAML::Value << "error";
        out << YAML::Key << "steps"  << YAML::Value << 0;
        out << YAML::Key << "score"  << YAML::Value << r.mission_score;
    }

    out << YAML::EndMap;
}

// Emits one mission block with all its (drone × lidar) run entries.
// Spec field order: mission_config, resolution_cm, resolution_request_status, runs.
void emitMission(YAML::Emitter& out, const std::vector<RunEntry>& mission_runs) {
    const RunEntry& first = mission_runs.front();
    const auto& cfg = first.result->mission_config;

    out << YAML::BeginMap;
    out << YAML::Key << "mission_config" << YAML::Value
        << (first.paths != nullptr ? first.paths->mission_config.string() : std::string{});
    out << YAML::Key << "resolution_cm" << YAML::Value << cfg.gps_resolution.numerical_value_in(cm);
    out << YAML::Key << "resolution_request_status" << YAML::Value
        << statusStr(first.result->resolution_request_status);

    out << YAML::Key << "runs" << YAML::Value << YAML::BeginSeq;
    for (const auto& e : mission_runs) {
        emitRun(out, e);
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
}

// Emits one simulation block, grouping its runs by mission (insertion order preserved).
void emitSimulation(YAML::Emitter& out, const std::vector<RunEntry>& sim_runs) {
    out << YAML::BeginMap;
    out << YAML::Key << "simulation_config" << YAML::Value << simKey(sim_runs.front());

    std::vector<std::string>           mission_order;
    std::vector<std::vector<RunEntry>> mission_groups;
    for (const auto& e : sim_runs) {
        const std::string mk = missionKey(e);
        auto it = std::find(mission_order.begin(), mission_order.end(), mk);
        if (it == mission_order.end()) {
            mission_order.push_back(mk);
            mission_groups.push_back({e});
        } else {
            mission_groups[static_cast<std::size_t>(std::distance(mission_order.begin(), it))]
                .push_back(e);
        }
    }

    out << YAML::Key << "missions" << YAML::Value << YAML::BeginSeq;
    for (const auto& group : mission_groups) {
        emitMission(out, group);
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
}

} // namespace

void SimulationReportWriter::write(const types::SimulationManagerReport& report,
                                   const std::filesystem::path& output_path,
                                   const std::filesystem::path& composition_file,
                                   const std::vector<RunPaths>& run_paths) {
    const Summary s = computeSummary(report.runs);

    // Pair each run with its source paths when the supplied paths align with the run count.
    const bool have_paths = run_paths.size() == report.runs.size();
    std::vector<RunEntry> entries;
    entries.reserve(report.runs.size());
    for (std::size_t i = 0; i < report.runs.size(); ++i) {
        entries.push_back(RunEntry{&report.runs[i], have_paths ? &run_paths[i] : nullptr});
    }

    // Group runs by simulation (insertion order preserved).
    std::vector<std::string>           sim_order;
    std::vector<std::vector<RunEntry>> sim_groups;
    for (const auto& e : entries) {
        const std::string sk = simKey(e);
        auto it = std::find(sim_order.begin(), sim_order.end(), sk);
        if (it == sim_order.end()) {
            sim_order.push_back(sk);
            sim_groups.push_back({e});
        } else {
            sim_groups[static_cast<std::size_t>(std::distance(sim_order.begin(), it))].push_back(e);
        }
    }

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "score_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value << composition_file.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << report.generated_at_utc;
    out << YAML::Key << "metric" << YAML::Value << report.metric;

    out << YAML::Key << "score_range" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range);
    out << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range);
    out << YAML::EndMap;

    out << YAML::Key << "error_score" << YAML::Value << report.error_score;

    out << YAML::Key << "summary" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "total_runs"    << YAML::Value << s.total_runs;
    out << YAML::Key << "scored_runs"   << YAML::Value << s.scored_runs;
    out << YAML::Key << "error_runs"    << YAML::Value << s.error_runs;
    out << YAML::Key << "average_score" << YAML::Value << s.avg_score;
    out << YAML::Key << "min_score"     << YAML::Value << s.min_score;
    out << YAML::Key << "max_score"     << YAML::Value << s.max_score;
    out << YAML::EndMap;

    out << YAML::Key << "simulations" << YAML::Value << YAML::BeginSeq;
    for (const auto& group : sim_groups) {
        emitSimulation(out, group);
    }
    out << YAML::EndSeq;

    out << YAML::EndMap; // score_report
    out << YAML::EndMap; // top-level document

    std::filesystem::create_directories(output_path);
    std::ofstream f{output_path / "simulation_output.yaml"};
    f << out.c_str() << "\n";
}

} // namespace drone_mapper
