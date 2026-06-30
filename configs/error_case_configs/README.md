# Error-case configurations

Config fixtures (no source code) that deliberately trigger failure-handling paths so you can verify
the report behaves per spec. Each failing scenario is scored `-1` with a specific error code and the
run continues with the remaining scenarios.

## Stage 2 — start-position validation

```bash
./drone_mapper_simulation configs/error_case_configs/start_validation_compositions.yaml ./out_start
cat ./out_start/simulation_output.yaml
```

| Trigger | Config | Expected |
|---|---|---|
| Start outside the map | `sim_start_outside_map.yaml` (start (1000,1000,1000), map `[0,50)`) | `score: -1`, `error_ref.code: START_OUTSIDE_MAP` |
| Start inside an occupied voxel | `sim_start_in_obstacle.yaml` (start (45,45,45) = the solid voxel) | `score: -1`, `error_ref.code: START_IN_OBSTACLE` |
| Start outside mission bounds | `sim_start_outside_bounds.yaml` (start (35,35,35)) + `mission_small_bounds.yaml` (`[0,10)`) | `score: -1`, `error_ref.code: START_OUTSIDE_MISSION_BOUNDS` |
| Valid scenario continues | `sim_good.yaml` + `mission_normal.yaml` | a **real** score (not `-1`) |

Expected totals: **4 runs → 3 error (`-1`) + 1 scored**.

Order of checks (in `SimulationRunImpl::run`): outside-map → outside-mission-bounds → in-obstacle
(the map check gates the obstacle lookup; the bounds check precedes the obstacle check).

## Notes on other failure modes

- **Factory / setup failure (e.g. missing map):** a bad/missing map file makes the factory throw →
  the whole simulation group is scored `-1` with `FACTORY_ERROR` (group-fill), other groups continue.
  (Handled by `SimulationManager`; not re-fixtured here.)
- **Mission boundary inverted (`min > max`):** scored `-1` with `MISSION_BOUNDARY_INVALID`, validated
  before the factory is called.
- **Runtime collision (`DRONE_HITS_OBSTACLE`):** currently **non-fatal** by design (the drone
  reroutes). Making it fatal is a later, separately-staged change.
