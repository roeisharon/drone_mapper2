#include <gtest/gtest.h>

#include <drone_mapper/utils/SimulationReportWriter.h>
#include <drone_mapper/Units.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>
#include <vector>

using drone_mapper::cm;
using drone_mapper::RunPaths;
using drone_mapper::types::SimulationResult;
using drone_mapper::types::SimulationManagerReport;
using drone_mapper::types::MissionRunResult;
using drone_mapper::types::MissionRunStatus;
using drone_mapper::types::ResolutionRequestStatus;

// Fixture (suite "SimulationReportWriter"): writes a report to a temp dir and parses it back.
class SimulationReportWriter : public ::testing::Test {
protected:
    std::filesystem::path out_dir_;

    void SetUp() override {
        out_dir_ = std::filesystem::temp_directory_path() / "simulation_report_writer_test";
        std::filesystem::remove_all(out_dir_);
        std::filesystem::create_directories(out_dir_);
    }
    void TearDown() override { std::filesystem::remove_all(out_dir_); }

    // Builds one run result. A non-empty err_code adds an error entry (and implies an error run).
    static SimulationResult makeResult(double score, MissionRunStatus status, std::size_t steps,
                                       const std::string& err_code = "") {
        SimulationResult r;
        r.mission_score = score;
        r.resolution_request_status = ResolutionRequestStatus::Accepted;
        r.mission_config.gps_resolution = 10.0 * cm;
        MissionRunResult mr;
        mr.status = status;
        mr.steps  = steps;
        if (!err_code.empty()) mr.errors.push_back({err_code, "message"});
        r.mission_results.push_back(mr);
        return r;
    }

    static SimulationManagerReport makeReport(std::vector<SimulationResult> runs) {
        return SimulationManagerReport{"2026-01-01T00:00:00Z", "output_map_accuracy",
                                       {0.0, 100.0}, -1, std::move(runs)};
    }

    static RunPaths paths(const std::string& sim, const std::string& mission,
                          const std::string& drone, const std::string& lidar) {
        return RunPaths{sim, mission, drone, lidar};
    }

    // Writes the report and returns the parsed score_report node.
    YAML::Node writeAndLoad(const SimulationManagerReport& report,
                            const std::vector<RunPaths>& run_paths = {}) {
        drone_mapper::SimulationReportWriter::write(report, out_dir_, "compositions/test.yaml", run_paths);
        return YAML::LoadFile((out_dir_ / "simulation_output.yaml").string())["score_report"];
    }
};

// Header

// The header carries the composition file, timestamp, metric, score_range, and error_score.
TEST_F(SimulationReportWriter, EmitsHeaderFields) {
    const auto sr = writeAndLoad(makeReport({makeResult(90.0, MissionRunStatus::Completed, 5)}));
    EXPECT_EQ(sr["composition_file"].as<std::string>(), "compositions/test.yaml");
    EXPECT_EQ(sr["generated_at_utc"].as<std::string>(), "2026-01-01T00:00:00Z");
    EXPECT_EQ(sr["metric"].as<std::string>(), "output_map_accuracy");
    EXPECT_DOUBLE_EQ(sr["score_range"]["min"].as<double>(), 0.0);
    EXPECT_DOUBLE_EQ(sr["score_range"]["max"].as<double>(), 100.0);
    EXPECT_EQ(sr["error_score"].as<int>(), -1);
}

// Summary

// scored_runs counts score>=0, error_runs counts score<0, total is their sum.
TEST_F(SimulationReportWriter, SummaryCountsScoredAndError) {
    const auto sr = writeAndLoad(makeReport({
        makeResult(90.0, MissionRunStatus::Completed, 5),
        makeResult(80.0, MissionRunStatus::MaxSteps, 9),
        makeResult(-1.0, MissionRunStatus::Error, 0, "DRONE_HITS_OBSTACLE"),
    }));
    const auto summary = sr["summary"];
    EXPECT_EQ(summary["total_runs"].as<int>(), 3);
    EXPECT_EQ(summary["scored_runs"].as<int>(), 2);
    EXPECT_EQ(summary["error_runs"].as<int>(), 1);
    EXPECT_DOUBLE_EQ(summary["average_score"].as<double>(), 85.0);
    EXPECT_DOUBLE_EQ(summary["min_score"].as<double>(), 80.0);
    EXPECT_DOUBLE_EQ(summary["max_score"].as<double>(), 90.0);
}

// Path-aware emission

// With run paths, the simulation block carries the simulation_config path.
TEST_F(SimulationReportWriter, EmitsSimulationConfigPath) {
    const auto sr = writeAndLoad(makeReport({makeResult(90.0, MissionRunStatus::Completed, 5)}),
                                 {paths("sims/s.yaml", "missions/m.yaml", "drones/d.yaml", "lidars/l.yaml")});
    EXPECT_EQ(sr["simulations"][0]["simulation_config"].as<std::string>(), "sims/s.yaml");
}

// The mission block carries its config path and resolution_cm.
TEST_F(SimulationReportWriter, EmitsMissionConfigPathAndResolution) {
    const auto sr = writeAndLoad(makeReport({makeResult(90.0, MissionRunStatus::Completed, 5)}),
                                 {paths("sims/s.yaml", "missions/m.yaml", "drones/d.yaml", "lidars/l.yaml")});
    const auto mission = sr["simulations"][0]["missions"][0];
    EXPECT_EQ(mission["mission_config"].as<std::string>(), "missions/m.yaml");
    EXPECT_DOUBLE_EQ(mission["resolution_cm"].as<double>(), 10.0);
    EXPECT_EQ(mission["resolution_request_status"].as<std::string>(), "Accepted");
}

// Each run carries its drone and lidar config paths.
TEST_F(SimulationReportWriter, EmitsDroneAndLidarConfigPaths) {
    const auto sr = writeAndLoad(makeReport({makeResult(90.0, MissionRunStatus::Completed, 5)}),
                                 {paths("sims/s.yaml", "missions/m.yaml", "drones/d.yaml", "lidars/l.yaml")});
    const auto run = sr["simulations"][0]["missions"][0]["runs"][0];
    EXPECT_EQ(run["drone_config"].as<std::string>(), "drones/d.yaml");
    EXPECT_EQ(run["lidar_config"].as<std::string>(), "lidars/l.yaml");
}

// Per-run fields

// A run emits status (lowercase), steps, and score.
TEST_F(SimulationReportWriter, EmitsStatusStepsScore) {
    const auto sr = writeAndLoad(makeReport({makeResult(93.5, MissionRunStatus::Completed, 1231)}));
    const auto run = sr["simulations"][0]["missions"][0]["runs"][0];
    EXPECT_EQ(run["status"].as<std::string>(), "completed");
    EXPECT_EQ(run["steps"].as<int>(), 1231);
    EXPECT_DOUBLE_EQ(run["score"].as<double>(), 93.5);
}

// max_steps status maps to the spec string "max_steps".
TEST_F(SimulationReportWriter, EmitsMaxStepsStatusString) {
    const auto sr = writeAndLoad(makeReport({makeResult(90.0, MissionRunStatus::MaxSteps, 2400)}));
    EXPECT_EQ(sr["simulations"][0]["missions"][0]["runs"][0]["status"].as<std::string>(), "max_steps");
}

// An error run emits an error_ref block with the error code.
TEST_F(SimulationReportWriter, EmitsErrorRefCodeOnError) {
    const auto sr = writeAndLoad(makeReport({
        makeResult(-1.0, MissionRunStatus::Error, 1320, "DRONE_HITS_OBSTACLE")}));
    const auto run = sr["simulations"][0]["missions"][0]["runs"][0];
    EXPECT_EQ(run["status"].as<std::string>(), "error");
    ASSERT_TRUE(run["error_ref"]);
    EXPECT_EQ(run["error_ref"]["code"].as<std::string>(), "DRONE_HITS_OBSTACLE");
}

// A successful run has no error_ref block.
TEST_F(SimulationReportWriter, NoErrorRefOnSuccess) {
    const auto sr = writeAndLoad(makeReport({makeResult(90.0, MissionRunStatus::Completed, 5)}));
    EXPECT_FALSE(sr["simulations"][0]["missions"][0]["runs"][0]["error_ref"]);
}

// Grouping

// Two runs sharing one mission (different drones) group under a single mission's runs list.
TEST_F(SimulationReportWriter, GroupsRunsUnderMissionByDrone) {
    const auto sr = writeAndLoad(
        makeReport({makeResult(90.0, MissionRunStatus::Completed, 5),
                    makeResult(80.0, MissionRunStatus::Completed, 6)}),
        {paths("sims/s.yaml", "missions/m.yaml", "drones/d1.yaml", "lidars/l.yaml"),
         paths("sims/s.yaml", "missions/m.yaml", "drones/d2.yaml", "lidars/l.yaml")});
    ASSERT_EQ(sr["simulations"].size(), 1u);
    ASSERT_EQ(sr["simulations"][0]["missions"].size(), 1u);
    EXPECT_EQ(sr["simulations"][0]["missions"][0]["runs"].size(), 2u);
}

// Distinct mission paths under one simulation produce separate mission blocks.
TEST_F(SimulationReportWriter, SeparatesDistinctMissions) {
    const auto sr = writeAndLoad(
        makeReport({makeResult(90.0, MissionRunStatus::Completed, 5),
                    makeResult(80.0, MissionRunStatus::Completed, 6)}),
        {paths("sims/s.yaml", "missions/m1.yaml", "drones/d.yaml", "lidars/l.yaml"),
         paths("sims/s.yaml", "missions/m2.yaml", "drones/d.yaml", "lidars/l.yaml")});
    ASSERT_EQ(sr["simulations"].size(), 1u);
    EXPECT_EQ(sr["simulations"][0]["missions"].size(), 2u);
}

// Distinct simulation paths produce separate simulation blocks.
TEST_F(SimulationReportWriter, SeparatesDistinctSimulations) {
    const auto sr = writeAndLoad(
        makeReport({makeResult(90.0, MissionRunStatus::Completed, 5),
                    makeResult(80.0, MissionRunStatus::Completed, 6)}),
        {paths("sims/a.yaml", "missions/m.yaml", "drones/d.yaml", "lidars/l.yaml"),
         paths("sims/b.yaml", "missions/m.yaml", "drones/d.yaml", "lidars/l.yaml")});
    EXPECT_EQ(sr["simulations"].size(), 2u);
}

// Fallback (no paths)

// Without run paths, simulations are grouped/identified by the map filename.
TEST_F(SimulationReportWriter, FallbackGroupsByMapFilenameNoPaths) {
    auto r1 = makeResult(90.0, MissionRunStatus::Completed, 5);
    auto r2 = makeResult(80.0, MissionRunStatus::Completed, 6);
    r1.simulation_config.map_filename = "maps/office.npy";
    r2.simulation_config.map_filename = "maps/office.npy";
    const auto sr = writeAndLoad(makeReport({r1, r2})); // no run_paths
    ASSERT_EQ(sr["simulations"].size(), 1u);
    EXPECT_EQ(sr["simulations"][0]["simulation_config"].as<std::string>(), "maps/office.npy");
}

// A run_paths vector whose size does not match the run count is ignored (fallback to map filename).
TEST_F(SimulationReportWriter, MismatchedPathCountFallsBack) {
    auto r1 = makeResult(90.0, MissionRunStatus::Completed, 5);
    auto r2 = makeResult(80.0, MissionRunStatus::Completed, 6);
    r1.simulation_config.map_filename = "maps/office.npy";
    r2.simulation_config.map_filename = "maps/office.npy";
    // Only one RunPaths for two runs → size mismatch → paths ignored.
    const auto sr = writeAndLoad(makeReport({r1, r2}),
                                 {paths("sims/s.yaml", "missions/m.yaml", "drones/d.yaml", "lidars/l.yaml")});
    EXPECT_EQ(sr["simulations"][0]["simulation_config"].as<std::string>(), "maps/office.npy");
}
