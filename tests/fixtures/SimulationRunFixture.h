#pragma once

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/mocks/MockGPS.h>
#include <drone_mapper/mocks/MockMovement.h>
#include <drone_mapper/simulation/SimulationRunImpl.h>
#include <drone_mapper/Units.h>

#include "mocks/MockIMissionControl.h"
#include "mocks/MockIMap3D.h"
#include "mocks/MockIMutableMap3D.h"
#include "mocks/MockIGPS.h"
#include "mocks/MockIDroneMovement.h"
#include "mocks/MockILidar.h"
#include "mocks/MockIMappingAlgorithm.h"
#include "mocks/MockIDroneControl.h"

#include <filesystem>
#include <memory>
#include <string>

// Shared fixture for the SimulationRun suite, used by two translation units:
//   - mock_gps_movement_test.cpp : real MockGPS + MockMovement behaviour (Step 3)
//   - simulation_run_test.cpp    : SimulationRunImpl::run() via injected mocks (Step 9a)
// GTest requires every test in a named suite to use the same fixture *type*, so this class is
// defined once here at global scope and both TUs include it.
//   GPS/movement tests use `gps`/`movement` directly and ignore the mock_* members.
//   SimulationRunImpl tests call makeRun(), then configure mock_hidden_/mock_output_/mock_mission_.
class SimulationRun : public ::testing::Test {
protected:
    // A real concrete MockGPS at world origin, heading 0deg, 1 cm GPS resolution (new 3-arg ctor).
    // GTest reconstructs the fixture before every test, so each starts from this clean state.
    drone_mapper::MockGPS gps{
        {0.0 * drone_mapper::x_extent[drone_mapper::cm],
         0.0 * drone_mapper::y_extent[drone_mapper::cm],
         0.0 * drone_mapper::z_extent[drone_mapper::cm]},
        {0.0 * drone_mapper::deg, 0.0 * drone_mapper::deg},
        1.0 * drone_mapper::cm};

    // MockMovement mutates the gps above via advance/elevate/rotate. Declared after gps so member
    // initialisation order matches (movement holds a reference to gps).
    drone_mapper::MockMovement movement{gps};

    // Non-owning pointers to the NiceMocks created inside makeRun(); valid until the returned
    // SimulationRunImpl is destroyed. Configure them with ON_CALL / EXPECT_CALL after makeRun().
    drone_mapper::mocks::MockIMissionControl* mock_mission_{nullptr};
    drone_mapper::mocks::MockIMap3D*          mock_hidden_{nullptr};
    drone_mapper::mocks::MockIMutableMap3D*   mock_output_{nullptr};

    // Default configs, populated by SetUp() with minimal valid values; override fields per-test.
    drone_mapper::types::SimulationConfigData sim_cfg_{};
    drone_mapper::types::MissionConfigData    mission_cfg_{};

    std::filesystem::path output_file_{"/tmp/drone_mapper_sim_run_test/output.npy"};

    void SetUp() override {
        sim_cfg_.map_filename = "test_map.npy";
        mission_cfg_.max_steps = 10;
        mission_cfg_.output_mapping_resolution_factor = 0.0;
    }

    // A MapConfig covering exactly one 1 cm^3 voxel at the origin. One voxel makes expected scores
    // trivial to reason about (0 or 100) in the comparison-based score tests.
    static drone_mapper::types::MapConfig singleVoxelConfig() {
        using namespace drone_mapper;
        return types::MapConfig{
            types::MappingBounds{
                0.0 * x_extent[cm], 1.0 * x_extent[cm],
                0.0 * y_extent[cm], 1.0 * y_extent[cm],
                0.0 * z_extent[cm], 1.0 * z_extent[cm]},
            Position3D{},
            1.0 * cm};
    }

    // Canned MissionRunResult values for the three terminal statuses.
    static drone_mapper::types::MissionRunResult completedResult() {
        return {drone_mapper::types::MissionRunStatus::Completed, 5, {}};
    }
    static drone_mapper::types::MissionRunResult maxStepsResult() {
        return {drone_mapper::types::MissionRunStatus::MaxSteps, 10, {}};
    }
    static drone_mapper::types::MissionRunResult errorResult(std::string msg = "motor_stall") {
        return {drone_mapper::types::MissionRunStatus::Error, 3,
                {{"DRONE_ERROR", std::move(msg)}}};
    }

    // Builds a SimulationRunImpl wired with NiceMock dependencies. After the call, mock_hidden_,
    // mock_output_, and mock_mission_ point to the injected mocks. Defaults (applied when an argument
    // is the zero-value for its type): sim -> sim_cfg_, mis -> mission_cfg_, path -> output_file_.
    // Hidden/output default to a single-voxel config with all-Unmapped voxels; mission defaults to a
    // clean Completed result. run() never drives gps/movement/lidar/algorithm/drone_control directly,
    // so those are silent NiceMocks owned only for lifetime.
    std::unique_ptr<drone_mapper::SimulationRunImpl> makeRun(
        drone_mapper::types::SimulationConfigData sim  = {},
        drone_mapper::types::MissionConfigData    mis  = {},
        std::filesystem::path                     path = {}) {
        if (sim.map_filename.empty())
            sim = sim_cfg_;
        if (mis.max_steps == 0 && mis.output_mapping_resolution_factor == 0.0)
            mis = mission_cfg_;
        if (path.empty())
            path = output_file_;

        using ::testing::NiceMock;
        using ::testing::Return;
        using ::testing::_;
        using drone_mapper::mocks::MockIMap3D;
        using drone_mapper::mocks::MockIMutableMap3D;
        using drone_mapper::mocks::MockIMissionControl;
        using drone_mapper::mocks::MockIGPS;
        using drone_mapper::mocks::MockIDroneMovement;
        using drone_mapper::mocks::MockILidar;
        using drone_mapper::mocks::MockIMappingAlgorithm;
        using drone_mapper::mocks::MockIDroneControl;
        using drone_mapper::types::VoxelOccupancy;

        auto hidden = std::make_unique<NiceMock<MockIMap3D>>();
        mock_hidden_ = hidden.get();
        ON_CALL(*mock_hidden_, getMapConfig()).WillByDefault(Return(singleVoxelConfig()));
        ON_CALL(*mock_hidden_, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Unmapped));

        auto output = std::make_unique<NiceMock<MockIMutableMap3D>>();
        mock_output_ = output.get();
        ON_CALL(*mock_output_, getMapConfig()).WillByDefault(Return(singleVoxelConfig()));
        ON_CALL(*mock_output_, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Unmapped));

        auto mission = std::make_unique<NiceMock<MockIMissionControl>>();
        mock_mission_ = mission.get();
        ON_CALL(*mock_mission_, runMission()).WillByDefault(Return(completedResult()));

        return std::make_unique<drone_mapper::SimulationRunImpl>(
            std::move(hidden),
            std::move(output),
            std::make_unique<NiceMock<MockIGPS>>(),
            std::make_unique<NiceMock<MockIDroneMovement>>(),
            std::make_unique<NiceMock<MockILidar>>(),
            std::make_unique<NiceMock<MockIMappingAlgorithm>>(),
            std::make_unique<NiceMock<MockIDroneControl>>(),
            std::move(mission),
            sim, mis, path);
    }
};
