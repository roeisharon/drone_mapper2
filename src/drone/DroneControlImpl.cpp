#include <drone_mapper/drone/DroneControlImpl.h>
#include <drone_mapper/utils/ScanResultToVoxels.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_mapper {

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

        // 5. Execute the requested movement, clamped to the drone's per-command limits.
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
                const PhysicalLength clamped =
                    (mv.distance < drone_.max_advance) ? mv.distance : drone_.max_advance;
                move_result = movement_.advance(clamped);
                break;
            }
            case types::MovementCommandType::Elevate: {
                // Elevate can be negative (descend); clamp magnitude, keep the sign.
                const double dist_cm    = mv.distance.numerical_value_in(cm);
                const double max_cm     = drone_.max_elevate.numerical_value_in(cm);
                const double sign       = (dist_cm >= 0.0) ? 1.0 : -1.0;
                const double clamped_cm = std::min(std::abs(dist_cm), max_cm) * sign;
                move_result = movement_.elevate(clamped_cm * cm);
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
