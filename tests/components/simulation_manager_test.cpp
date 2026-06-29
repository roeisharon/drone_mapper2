#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/simulation/SimulationManager.h>
#include <drone_mapper/Units.h>

#include "mocks/MockISimulationRunFactory.h"
#include "mocks/MockISimulationRun.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace ::testing;
using drone_mapper::cm;
using drone_mapper::types::SimulationCompositionData;
using drone_mapper::types::SimulationConfigData;
using drone_mapper::types::MissionConfigData;
using drone_mapper::types::DroneConfigData;
using drone_mapper::types::LidarConfigData;
using drone_mapper::types::SimulationResult;
using drone_mapper::types::MissionRunResult;
using drone_mapper::types::MissionRunStatus;
using drone_mapper::types::MappingBounds;

// Fixture (suite "SimulationManager"): a manager wired to a NiceMock factory, plus a temp output dir.
class SimulationManager : public ::testing::Test {
protected:
    drone_mapper::mocks::MockISimulationRunFactory* factory_{nullptr};
    std::unique_ptr<drone_mapper::SimulationManager> manager_;
    std::filesystem::path out_dir_;

    void SetUp() override {
        out_dir_ = std::filesystem::temp_directory_path() / "simulation_manager_test";
        std::filesystem::remove_all(out_dir_);

        auto factory = std::make_unique<NiceMock<drone_mapper::mocks::MockISimulationRunFactory>>();
        factory_ = factory.get();
        manager_ = std::make_unique<drone_mapper::SimulationManager>(std::move(factory));

        // By default every create() returns a run that scores 90 on a completed 1-step mission.
        ON_CALL(*factory_, create(_, _, _, _, _))
            .WillByDefault([](const SimulationConfigData&, const MissionConfigData&,
                              const DroneConfigData&, const LidarConfigData&,
                              const std::filesystem::path&) {
                return makeRun(makeResult(90.0, MissionRunStatus::Completed, 1));
            });
    }
    void TearDown() override { std::filesystem::remove_all(out_dir_); }

    // Builds a SimulationResult with the given score/status/steps.
    static SimulationResult makeResult(double score, MissionRunStatus status, std::size_t steps) {
        SimulationResult r;
        r.mission_score = score;
        r.mission_results.push_back(MissionRunResult{status, steps, {}});
        return r;
    }

    // Wraps a SimulationResult in a NiceMock run object whose run() returns it.
    static std::unique_ptr<drone_mapper::ISimulationRun> makeRun(SimulationResult result) {
        auto run = std::make_unique<NiceMock<drone_mapper::mocks::MockISimulationRun>>();
        ON_CALL(*run, run()).WillByDefault(Return(result));
        return run;
    }

    // Builds a composition with `sims` groups (each with `missions` missions) plus `drones` drones
    // and `lidars` lidars shared globally.
    static SimulationCompositionData makeComposition(int sims, int missions, int drones, int lidars) {
        SimulationCompositionData comp;
        comp.composition_file = "compositions/test.yaml";
        for (int s = 0; s < sims; ++s) {
            SimulationConfigData sim;
            sim.map_filename = "map_" + std::to_string(s) + ".npy";
            std::vector<MissionConfigData> ms;
            for (int m = 0; m < missions; ++m) {
                MissionConfigData mc;
                mc.max_steps = static_cast<std::size_t>(100 + m);
                mc.gps_resolution = 10.0 * cm;
                ms.push_back(mc);
            }
            comp.simulation_mission_groups.emplace_back(sim, std::move(ms));
        }
        for (int d = 0; d < drones; ++d) comp.drones.push_back(DroneConfigData{});
        for (int l = 0; l < lidars; ++l) comp.lidars.push_back(LidarConfigData{});
        return comp;
    }

    std::string readReport() const {
        std::ifstream f(out_dir_ / "simulation_output.yaml");
        return {std::istreambuf_iterator<char>(f), {}};
    }
};

// Constructor

// A null factory is rejected at construction.
TEST_F(SimulationManager, ConstructorThrowsOnNullFactory) {
    EXPECT_THROW((drone_mapper::SimulationManager{nullptr}), std::invalid_argument);
}

// Cartesian product call counts

// One sim × one mission × one drone × one lidar → exactly one create() call.
TEST_F(SimulationManager, SingleComboCallsFactoryOnce) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(1);
    (void)manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
}

// Two simulation groups → two create() calls.
TEST_F(SimulationManager, TwoSimsCallTwice) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(2);
    (void)manager_->run(makeComposition(2, 1, 1, 1), out_dir_);
}

// Two missions in one group → two create() calls.
TEST_F(SimulationManager, TwoMissionsCallTwice) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(2);
    (void)manager_->run(makeComposition(1, 2, 1, 1), out_dir_);
}

// Two drones → two create() calls.
TEST_F(SimulationManager, TwoDronesCallTwice) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(2);
    (void)manager_->run(makeComposition(1, 1, 2, 1), out_dir_);
}

// Two lidars → two create() calls.
TEST_F(SimulationManager, TwoLidarsCallTwice) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(2);
    (void)manager_->run(makeComposition(1, 1, 1, 2), out_dir_);
}

// Full product 2×2×2×2 = 16 create() calls and 16 runs.
TEST_F(SimulationManager, FullCartesianSixteenCalls) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(16);
    const auto report = manager_->run(makeComposition(2, 2, 2, 2), out_dir_);
    EXPECT_EQ(report.runs.size(), 16u);
}

// An empty composition runs nothing and returns no runs.
TEST_F(SimulationManager, EmptySimulationsNoCallsEmptyRuns) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(0);
    const auto report = manager_->run(makeComposition(0, 0, 0, 0), out_dir_);
    EXPECT_TRUE(report.runs.empty());
}

// Run delegation and result propagation

// run() is invoked on each created run object, and its score lands in the report.
TEST_F(SimulationManager, ScoreMatchesRunObjectReturn) {
    ON_CALL(*factory_, create(_, _, _, _, _))
        .WillByDefault([](const SimulationConfigData&, const MissionConfigData&,
                          const DroneConfigData&, const LidarConfigData&,
                          const std::filesystem::path&) {
            return makeRun(makeResult(42.5, MissionRunStatus::Completed, 3));
        });
    const auto report = manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, 42.5);
}

// The simulation config flows to the factory unchanged.
TEST_F(SimulationManager, SimulationConfigPassedToFactory) {
    auto comp = makeComposition(1, 1, 1, 1);
    std::get<0>(comp.simulation_mission_groups[0]).map_filename = "specific_map.npy";
    EXPECT_CALL(*factory_, create(Field(&SimulationConfigData::map_filename,
                                        std::filesystem::path{"specific_map.npy"}),
                                  _, _, _, _))
        .Times(1);
    (void)manager_->run(comp, out_dir_);
}

// The mission config is preserved on the returned result.
TEST_F(SimulationManager, MissionConfigPreserved) {
    ON_CALL(*factory_, create(_, _, _, _, _))
        .WillByDefault([](const SimulationConfigData&, const MissionConfigData& mission,
                          const DroneConfigData&, const LidarConfigData&,
                          const std::filesystem::path&) {
            auto r = makeResult(50.0, MissionRunStatus::Completed, 1);
            r.mission_config = mission; // SimulationRunImpl echoes the mission config
            return makeRun(r);
        });
    auto comp = makeComposition(1, 1, 1, 1);
    std::get<1>(comp.simulation_mission_groups[0])[0].max_steps = 777;
    const auto report = manager_->run(comp, out_dir_);
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_config.max_steps, 777u);
}

// The output path is forwarded to the factory.
TEST_F(SimulationManager, OutputPathPassedToFactory) {
    EXPECT_CALL(*factory_, create(_, _, _, _, out_dir_)).Times(1);
    (void)manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
}

// Report header fields

// The metric is fixed to output_map_accuracy.
TEST_F(SimulationManager, MetricIsOutputMapAccuracy) {
    EXPECT_EQ(manager_->run(makeComposition(1, 1, 1, 1), out_dir_).metric, "output_map_accuracy");
}

// The score range is [0, 100].
TEST_F(SimulationManager, ScoreRangeIs0To100) {
    const auto report = manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
}

// The error score sentinel is -1.
TEST_F(SimulationManager, ErrorScoreIsNegativeOne) {
    EXPECT_EQ(manager_->run(makeComposition(1, 1, 1, 1), out_dir_).error_score, -1);
}

// The timestamp is non-empty and looks like ISO-8601 UTC (has 'T', ends with 'Z').
TEST_F(SimulationManager, TimestampHasTandZ) {
    const auto ts = manager_->run(makeComposition(1, 1, 1, 1), out_dir_).generated_at_utc;
    ASSERT_FALSE(ts.empty());
    EXPECT_NE(ts.find('T'), std::string::npos);
    EXPECT_EQ(ts.back(), 'Z');
}

// Factory-exception / group-failure semantics

// A factory exception yields a score -1 for that run instead of propagating.
TEST_F(SimulationManager, FactoryExceptionSetsScoreNegativeOne) {
    ON_CALL(*factory_, create(_, _, _, _, _))
        .WillByDefault(Throw(std::runtime_error("bad map file")));
    const auto report = manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
}

// After the first factory throw in a group, the rest of that group is filled WITHOUT calling create
// again — so a 1×1×1×3 group throwing on the first call makes exactly one create() call.
TEST_F(SimulationManager, FactoryExceptionGroupFailsRestWithoutCalling) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _))
        .Times(1)
        .WillOnce(Throw(std::runtime_error("bad map file")));
    const auto report = manager_->run(makeComposition(1, 1, 1, 3), out_dir_);
    ASSERT_EQ(report.runs.size(), 3u); // all three combos present
    for (const auto& r : report.runs) EXPECT_DOUBLE_EQ(r.mission_score, -1.0);
}

// A failed group does not affect a later simulation group, which runs normally.
TEST_F(SimulationManager, FactoryExceptionDoesNotAffectOtherSimGroups) {
    // First group's create throws; subsequent create succeeds (score 88).
    EXPECT_CALL(*factory_, create(_, _, _, _, _))
        .WillOnce(Throw(std::runtime_error("group 0 bad")))
        .WillRepeatedly([](const SimulationConfigData&, const MissionConfigData&,
                           const DroneConfigData&, const LidarConfigData&,
                           const std::filesystem::path&) {
            return makeRun(makeResult(88.0, MissionRunStatus::Completed, 1));
        });
    const auto report = manager_->run(makeComposition(2, 1, 1, 1), out_dir_);
    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0); // failed group
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 88.0); // healthy group
}

// A factory exception is logged to the manager error log, with the exception message.
TEST_F(SimulationManager, FactoryExceptionCreatesErrorLog) {
    ON_CALL(*factory_, create(_, _, _, _, _))
        .WillByDefault(Throw(std::runtime_error("explode_message")));
    (void)manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    const auto log = out_dir_ / "simulation_manager_errors.log";
    ASSERT_TRUE(std::filesystem::exists(log));
    std::ifstream f(log);
    const std::string content{std::istreambuf_iterator<char>(f), {}};
    EXPECT_NE(content.find("explode_message"), std::string::npos);
}

// A failed scenario is scored -1 AND carries an error code + descriptive message (the cause), not a
// blank result — so the report shows why it failed.
TEST_F(SimulationManager, FactoryExceptionResultCarriesErrorCode) {
    ON_CALL(*factory_, create(_, _, _, _, _))
        .WillByDefault(Throw(std::runtime_error("Failed to load NPY file: missing.npy")));
    const auto report = manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    ASSERT_FALSE(report.runs[0].mission_results[0].errors.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].errors.front().code, "FACTORY_ERROR");
    EXPECT_NE(report.runs[0].mission_results[0].errors.front().message.find("missing.npy"),
              std::string::npos);
}

// When a whole group is auto-filled after a setup failure, EVERY filled scenario carries the same
// error code (not a blank one), so the group's failure is fully explained in the report.
TEST_F(SimulationManager, GroupFillScenariosCarryErrorCode) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _))
        .Times(1)
        .WillOnce(Throw(std::runtime_error("bad map file")));
    const auto report = manager_->run(makeComposition(1, 1, 1, 3), out_dir_);
    ASSERT_EQ(report.runs.size(), 3u);
    for (const auto& r : report.runs) {
        EXPECT_DOUBLE_EQ(r.mission_score, -1.0);
        ASSERT_FALSE(r.mission_results.empty());
        ASSERT_FALSE(r.mission_results[0].errors.empty());
        EXPECT_EQ(r.mission_results[0].errors.front().code, "FACTORY_ERROR");
    }
}

// Mission boundary validation

// A mission with inverted boundaries fails all its combos with MISSION_BOUNDARY_INVALID and never
// calls the factory.
TEST_F(SimulationManager, InvertedMissionBoundsFailsWithoutCallingFactory) {
    EXPECT_CALL(*factory_, create(_, _, _, _, _)).Times(0);
    auto comp = makeComposition(1, 1, 1, 1);
    MappingBounds bad; // min > max on x
    bad.min_x = 50.0 * drone_mapper::x_extent[cm];
    bad.max_x = 10.0 * drone_mapper::x_extent[cm];
    std::get<1>(comp.simulation_mission_groups[0])[0].mission_bounds = bad;
    const auto report = manager_->run(comp, out_dir_);
    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].errors.front().code, "MISSION_BOUNDARY_INVALID");
}

// Output side-effects

// run() creates the output directory if it does not already exist.
TEST_F(SimulationManager, RunCreatesOutputDirectory) {
    ASSERT_FALSE(std::filesystem::exists(out_dir_));
    (void)manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    EXPECT_TRUE(std::filesystem::exists(out_dir_));
}

// run() writes simulation_output.yaml.
TEST_F(SimulationManager, YamlOutputFileCreated) {
    (void)manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    EXPECT_TRUE(std::filesystem::exists(out_dir_ / "simulation_output.yaml"));
}

// The emitted YAML carries the metric and composition file.
TEST_F(SimulationManager, YamlContainsMetricAndCompositionFile) {
    (void)manager_->run(makeComposition(1, 1, 1, 1), out_dir_);
    const auto txt = readReport();
    EXPECT_NE(txt.find("output_map_accuracy"), std::string::npos);
    EXPECT_NE(txt.find("compositions/test.yaml"), std::string::npos);
}

// The emitted YAML carries the score range, summary, and timestamp.
TEST_F(SimulationManager, YamlContainsScoreRangeAndSummary) {
    (void)manager_->run(makeComposition(2, 1, 1, 1), out_dir_);
    const auto txt = readReport();
    EXPECT_NE(txt.find("score_range"), std::string::npos);
    EXPECT_NE(txt.find("total_runs"), std::string::npos);
    EXPECT_NE(txt.find("generated_at_utc"), std::string::npos);
}
