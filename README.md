# Assignment 2 Refactor Skeleton

This repository is a compilable skeleton for Assignment 2 in the 2026 Advanced
Topics in Programming course. It intentionally provides interfaces, data types,
dependency-injected component stubs, and a preserved mock LiDAR implementation.
It **does not** implement the full simulator or mapping solution. You should not use ANY implementations provided in this repository (aside MockLidar).

## Project Structure

```text
include/drone_mapper/      Public interfaces, data types, and skeleton classes
src/                     Stub implementations and executable entry points
data_maps/               Example NumPy voxel maps
.devcontainer/           Development container setup
CMakeLists.txt           CMake build configuration
vcpkg.json               Dependency list
```


## Building

```bash
cmake --preset default
cmake --build --preset default
```

The main build targets are:

```text
drone_mapper_simulation
drone_mapper_simulation_test
maps_comparison
```

## Running

Simulator skeleton:

```bash
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
```

The skeleton wires explicit placeholder components and reports stub results.
You should add YAML parsing, scenario composition, output writing, error
logging, and real simulator behavior etc..

Maps comparison skeleton:

```bash
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

The provided `MapsComparison` implementation is only a placeholder. You
should replace it with the required scoring behavior.
