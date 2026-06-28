# Assignment 2 – Additional Forum Updates (Latest Clarifications)

This document summarizes the latest official forum updates that should be incorporated into the project documentation and considered while implementing the system.

---

# 1. Maps Must Respect Mission Boundaries

## Official clarification

When creating maps (including mocks used in unit tests):

* The map should at least cover the configured mapping boundaries.
* `set()` should **never** be called for a voxel outside the mapping boundaries.
* The underlying `NpyArray` should also be validated against its shape to prevent out-of-bounds access.

The course staff suggested that maps should generally be allocated up front since the mission boundaries are already known.

## Design implications

* Treat map boundaries as strict invariants.
* Never rely on dynamic map growth.
* Validate voxel indices before accessing the underlying array.
* Unit-test mocks should faithfully represent valid maps.

---

# 2. Freedom to Modify the `include/` Directory

## Official clarification

You are allowed to:

* Modify existing header files.
* Add new header files.
* Add helper declarations.
* Add additional `#include`s.

The only restriction is:

* **Do not change public interfaces.**

Your implementation should still work if other provided implementations are used.

## Design implications

* Prefer interface-based dependencies.
* Avoid introducing unnecessary coupling between modules.
* Helper abstractions and utility headers are encouraged.

---

# 3. README Requirements

## Official clarification

You may:

* Continue using the existing `README.md` from the skeleton.
* Or create a completely new README.

The important requirement is that the documentation remains accurate.

It should document:

* `simulation_output.yaml`
* output folder format
* any additional build or usage instructions introduced by your implementation

## Action

Review the README before submission and ensure it reflects the final implementation.

---

# 4. Lidar Now Exposes Its Configuration

## Background

A previous API update required `ScanResultToVoxels` to receive `LidarConfigData`.

A follow-up question identified that `DroneControlImpl` had no way to retrieve that configuration from the lidar.

## Official fix

The course staff will add:

* a getter for `LidarConfigData` inside the lidar interface.

## Design implications

Do not implement custom workarounds.

Use the official getter once the updated skeleton is available.

---

# 5. ScanResultToVoxels Does NOT Need to Be Mocked

## Official clarification

`ScanResultToVoxels::applyToMap()` remains a static utility.

There is **no requirement** to mock it in component tests.

The production implementation may be used directly.

If desired, an optional template-based dependency injection pattern may be used to substitute an alternative implementation during testing.

This is entirely optional.

## Design implications

* Treat `ScanResultToVoxels` as a trusted utility.
* Component tests are not expected to isolate it.
* Do not over-engineer wrappers solely for mocking purposes unless they provide additional value.

---

# 6. MappingBounds Were Restored to MissionConfig

## Official clarification

`MappingBounds` has been added back to `MissionConfig`.

It also remains part of `MapConfig`.

## Design implications

Mission-level logic can once again access mapping boundaries directly through the mission configuration.

Any previous workarounds based on the temporary API should be removed.

---

# 7. SimulationCompositionData Hierarchy Changed

## Official clarification

The YAML hierarchy is now reflected by the API.

Previously:

```text
SimulationCompositionData
    simulations
    missions
```

Now:

Each `SimulationConfigData` is paired with its own list of `MissionConfigData`.

Conceptually:

```text
Simulation
    ├── Mission
    ├── Mission
    └── Mission
```

instead of a flat cross-product.

## Design implications

Update parsing and execution logic to preserve this hierarchy.

Do not assume every mission belongs to every simulation.

---

# Implementation Notes

These updates supersede previous assumptions in several areas:

* Maps are expected to have predefined dimensions.
* Boundary validation is now an explicit design requirement.
* `MappingBounds` are once again available through `MissionConfig`.
* The simulation/mission hierarchy has changed and should be reflected throughout the architecture.
* `ScanResultToVoxels` should be treated as a production utility rather than a dependency that must be mocked.
* The updated lidar API should be used instead of introducing custom solutions for accessing `LidarConfigData`.

As with previous updates, review the latest skeleton before implementation, since the course staff continues to evolve the API as issues are discovered.
