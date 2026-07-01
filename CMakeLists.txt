cmake_minimum_required(VERSION 3.20)
project(drone_mapper LANGUAGES CXX)

find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

add_library(drone_mapper
    src/simulation/SimulationManager.cpp
    src/simulation/SimulationRunFactoryImpl.cpp
    src/simulation/SimulationRunImpl.cpp
    src/mission/MissionControlImpl.cpp
    src/drone/DroneControlImpl.cpp
    src/drone/MappingAlgorithmImpl.cpp
    src/map/Map3DImpl.cpp
    src/mocks/MockGPS.cpp
    src/mocks/MockLidar.cpp
    src/mocks/MockMovement.cpp
    src/utils/ScanResultToVoxels.cpp
    src/utils/MapsComparison.cpp
    src/utils/ErrorLogger.cpp
    src/utils/YamlConfigParser.cpp
    src/utils/SimulationReportWriter.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)

target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)

add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

add_executable(drone_mapper_simulation_test
    # Required component suites (per the assignment) + Map3DImpl (kept as a core-infrastructure
    # exception: it is the map implementation MappingAlgorithm/MockLidar/MapsComparison build on).
    tests/components/map3d_test.cpp
    tests/components/mock_lidar_test.cpp
    tests/components/mock_gps_movement_test.cpp
    tests/components/mapping_algorithm_test.cpp
    tests/components/drone_control_test.cpp
    tests/components/mission_control_test.cpp
    tests/components/simulation_run_test.cpp
    tests/components/simulation_manager_test.cpp
    tests/components/maps_comparison_test.cpp
    tests/integration/integration_test.cpp
    tests/integration/mock_algorithm_integration_test.cpp
)
target_compile_features(drone_mapper_simulation_test PUBLIC cxx_std_20)
target_compile_options(drone_mapper_simulation_test
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper_simulation_test PRIVATE tests/)
target_link_libraries(drone_mapper_simulation_test PRIVATE
    drone_mapper
    GTest::gtest
    GTest::gtest_main
    GTest::gmock
)
