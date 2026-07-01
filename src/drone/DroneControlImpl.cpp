#include <drone_mapper/drone/DroneControlImpl.h>
#include <drone_mapper/utils/ScanResultToVoxels.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_mapper {

namespace {

// Clamps a requested move to a WHOLE number of grid cells that fits within the per-command limit, so
// a move starting at a cell centre lands exactly on another cell centre (planner/execution geometry
// agree). Returns the distance in cm: min(requested_cells, floor(max/res)) * res. With res <= 0 it
// degrades to the plain continuous clamp.
//
// Both the requested and the max distance are floored (never rounded up), so the executed move is
// GUARANTEED never to exceed the requested/validated distance — a half-cell request (e.g. 15cm at
// 10cm resolution) yields 1 whole cell, not 2. A tiny epsilon absorbs floating-point noise so an
// exact N*res request still floors to N rather than N-1. For requests that are integer multiples of
// the resolution (the common case, since the planner targets cell centres) the result is unchanged.
double cellAlignedDistanceCm(double requested_cm, double max_cm, double res_cm) {
    if (res_cm <= 0.0) {
        return std::min(requested_cm, max_cm);
    }
    constexpr double kCellEps = 1e-6;
    const long long req_cells = static_cast<long long>(std::floor(requested_cm / res_cm + kCellEps));
    const long long max_cells = static_cast<long long>(std::floor(max_cm / res_cm + kCellEps));
    const long long cells = std::min(req_cells, std::max<long long>(0, max_cells));
    return static_cast<double>(cells) * res_cm;
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    try {
        // 1. Read the current GPS pose.
        const Position3D  pos     = gps_.position();
        const Orientation heading = gps_.heading();

        // 2. Defensive obstacle check: if the drone's current cell is Occupied it has entered a
        //    wall (e.g. bad initial placement). Report it without advancing the step index.
        if (output_map_.atVoxel(pos) == types::VoxelOccupancy::Occupied) {
            return {types::DroneStepStatus::Error, "DRONE_HITS_OBSTACLE"};
        }

        // 2b. The drone physically occupies this cell and it is provably not Occupied, so mark it
        //     Empty. This guarantees every traversed cell is mapped even in open space, where the
        //     forward beams miss (no surface) and would otherwise leave the path Unmapped.
        output_map_.set(pos, types::VoxelOccupancy::Empty);

        // 3. Ask the algorithm what to do, passing the previous step's scan (nullptr on first call).
        const types::LidarScanResult* scan_ptr =
            latest_scan_result_.has_value() ? &*latest_scan_result_ : nullptr;
        const types::DroneState current_state{pos, heading, step_index_};
        const types::MappingStepCommand cmd = mapping_algorithm_.nextStep(current_state, scan_ptr);

        // 4. Algorithm completion → Completed (both finish statuses are terminal-success).
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            ++step_index_;
            return {types::DroneStepStatus::Completed, {}};
        }

        // 5. Execute the requested movement. Translation commands are clamped to a WHOLE number of
        //    grid cells within the per-command limit so the drone lands exactly on a cell centre
        //    (Checkpoint A — planner and execution validate the same footprint). Rotation is an angle,
        //    so it keeps the plain continuous clamp.
        const double res_cm = output_map_.getMapConfig().resolution.numerical_value_in(cm);
        types::MovementResult move_result{true, {}};
        if (cmd.movement.has_value()) {
            const auto& mv = *cmd.movement;
            switch (mv.type) {
            case types::MovementCommandType::Hover:
                break; // no-op; drone stays in place
            case types::MovementCommandType::Rotate: {
                const HorizontalAngle clamped =
                    (mv.angle < drone_.max_rotate) ? mv.angle : drone_.max_rotate;
                move_result = movement_.rotate(mv.rotation, clamped);
                break;
            }
            case types::MovementCommandType::Advance: {
                const double dist_cm = cellAlignedDistanceCm(
                    mv.distance.numerical_value_in(cm),
                    drone_.max_advance.numerical_value_in(cm), res_cm);
                move_result = movement_.advance(dist_cm * cm);
                break;
            }
            case types::MovementCommandType::Elevate: {
                // Elevate can be negative (descend); clamp the magnitude to whole cells, keep the sign.
                const double dist_cm = mv.distance.numerical_value_in(cm);
                const double sign    = (dist_cm >= 0.0) ? 1.0 : -1.0;
                const double mag_cm  = cellAlignedDistanceCm(
                    std::abs(dist_cm), drone_.max_elevate.numerical_value_in(cm), res_cm);
                move_result = movement_.elevate(sign * mag_cm * cm);
                break;
            }
            } // switch
        }

        // 6. Perform the scan (if requested) from the post-movement pose, then hand it to the
        //    staff converter which writes the observed voxels into the output map. Store the scan
        //    so the next nextStep() can receive it as latest_scan.
        if (cmd.scan_orientation.has_value()) {
            const types::LidarScanResult new_scan = lidar_.scan(*cmd.scan_orientation);
            latest_scan_result_ = new_scan;
            const Position3D  scan_pos     = gps_.position();
            const Orientation scan_heading = gps_.heading();
            ScanResultToVoxels::applyToMap(output_map_, scan_pos, scan_heading, new_scan, lidar_.config());
        }

        // 7. Advance the step counter. A collision refusal (DRONE_HITS_OBSTACLE) is a routine part
        //    of exploration, NOT a mission failure: the optimistic planner probes an unmapped cell
        //    the movement layer knows is solid/too-narrow, the drone simply stays put, and the next
        //    scan maps that obstacle so the planner reroutes around it. Any OTHER movement failure
        //    is a genuine fault and still surfaces as a step error.
        ++step_index_;
        if (!move_result.success) {
            if (move_result.message == "DRONE_HITS_OBSTACLE") {
                return {types::DroneStepStatus::Continue, {}};
            }
            return {types::DroneStepStatus::Error, move_result.message};
        }
        return {types::DroneStepStatus::Continue, {}};

    } catch (const std::exception& e) {
        return {types::DroneStepStatus::Error,
                std::string("DroneControlImpl::step exception: ") + e.what()};
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
