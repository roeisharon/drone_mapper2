#include <drone_mapper/drone/MappingAlgorithmImpl.h>

#include <drone_mapper/map/IMap3D.h>
#include <drone_mapper/utils/DroneFootprint.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <optional>

namespace drone_mapper {

namespace {

// Minimum heading difference (degrees) that triggers a Rotate command rather than an Advance.
constexpr double kAngleTolDeg = 0.5;

// Normalises an angle difference to (-180, +180].
double normaliseDiff(double d) noexcept {
    while (d >  180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

} // namespace

std::size_t MappingAlgorithmImpl::GridCellHash::operator()(const GridCell& c) const noexcept {
    std::size_t h = std::hash<int>{}(c.x);
    h ^= std::hash<int>{}(c.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(c.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
    return h;
}

double MappingAlgorithmImpl::stepCm() const noexcept {
    const double step = output_map_.getMapConfig().resolution.numerical_value_in(cm);
    return (step > 0.0) ? step : 1.0;
}

MappingAlgorithmImpl::GridCell MappingAlgorithmImpl::worldToGrid(const Position3D& pos) const noexcept {
    const double res = stepCm();
    const auto& config = output_map_.getMapConfig();
    const double ox = config.offset.x.numerical_value_in(cm);
    const double oy = config.offset.y.numerical_value_in(cm);
    const double oz = config.offset.z.numerical_value_in(cm);
    return {
        static_cast<int>(std::floor((pos.x.numerical_value_in(cm) + ox) / res)),
        static_cast<int>(std::floor((pos.y.numerical_value_in(cm) + oy) / res)),
        static_cast<int>(std::floor((pos.z.numerical_value_in(cm) + oz) / res)),
    };
}

Position3D MappingAlgorithmImpl::gridToWorld(const GridCell& cell) const noexcept {
    const double res = stepCm();
    const auto& config = output_map_.getMapConfig();
    const double ox = config.offset.x.numerical_value_in(cm);
    const double oy = config.offset.y.numerical_value_in(cm);
    const double oz = config.offset.z.numerical_value_in(cm);
    return {
        ((static_cast<double>(cell.x) + 0.5) * res - ox) * cm,
        ((static_cast<double>(cell.y) + 0.5) * res - oy) * cm,
        ((static_cast<double>(cell.z) + 0.5) * res - oz) * cm,
    };
}

bool MappingAlgorithmImpl::isNavigable(const GridCell& cell) const noexcept {
    // Navigable only if INSIDE the map and not a CONFIRMED obstacle. Unmapped means not yet scanned
    // → passable (optimistic) so exploration can proceed. OutOfBounds must NOT be navigable:
    // atVoxel() returns OutOfBounds (not Occupied) past the map edge, so without this the frontier
    // leaks into the infinite empty space outside the map and the algorithm never returns Finished.
    //
    // PotentiallyOccupied is treated as PASSABLE uncertainty, NOT a hard obstacle: it only means a
    // beam hit something closer than the lidar's z_min, so the exact voxel is unknown — not that a
    // wall is confirmed there. Blocking on it stranded large drones near the start, whose big
    // footprint always overlaps the near-field PotentiallyOccupied shell the first scans paint (this
    // is why drone_large + long-range lidar terminated after ~3 steps). The real collision guard is
    // MockMovement, which validates every move against the hidden ground-truth map (and refuses a
    // genuine wall non-fatally); the next scan then resolves the uncertain voxel to Empty/Occupied.
    const Position3D center = gridToWorld(cell);
    const types::VoxelOccupancy v = output_map_.atVoxel(center);
    if (v == types::VoxelOccupancy::Occupied ||
        v == types::VoxelOccupancy::OutOfBounds) {
        return false;
    }
    // The cell itself is free, but the drone is a sphere of drone_config_.radius — it can only
    // occupy this cell if its full footprint clears every CONFIRMED obstacle. This stops the planner
    // routing a large drone through gaps a point-sized drone could slip through. With a zero radius
    // (default config) this reduces to the single-cell test above.
    return footprintFits(output_map_, center, drone_config_.radius,
                         [](types::VoxelOccupancy occ) {
                             return occ == types::VoxelOccupancy::Occupied;
                         });
}

// void MappingAlgorithmImpl::expandNeighbors(const GridCell& cell) {
//     const GridCell neighbours[6] = {
//         {cell.x + 1, cell.y,     cell.z    },
//         {cell.x - 1, cell.y,     cell.z    },
//         {cell.x,     cell.y + 1, cell.z    },
//         {cell.x,     cell.y - 1, cell.z    },
//         {cell.x,     cell.y,     cell.z + 1},
//         {cell.x,     cell.y,     cell.z - 1},
//     };
//     for (const GridCell& nb : neighbours) {
//         if (!visited_.count(nb) && !in_frontier_.count(nb) && isNavigable(nb)) {
//             frontier_.push(nb);
//             in_frontier_.insert(nb);
//         }
//     }
// }

types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const types::DroneState& state,
    const types::LidarScanResult* /*latest_scan*/)
{
    const GridCell current = worldToGrid(state.position);
    visited_.insert(current);
    
    // --- 1. STATELESS STUCK DETECTOR ---
    bool did_not_move = false;
    bool last_was_rotate = false;
    GridCell last_immediate_target;
    
    // Pull the 3-element state buffer from last tick
    if (frontier_.size() == 3) {
        last_immediate_target = frontier_.front(); frontier_.pop();
        GridCell flags = frontier_.front(); frontier_.pop();
        GridCell last_current = frontier_.front(); frontier_.pop();
        
        last_was_rotate = (flags.x == 1);
        if (last_current == current) {
            did_not_move = true;
        }
    }
    while(!frontier_.empty()) frontier_.pop(); // Clear queue safely

    // If we commanded a physical move but didn't actually move, we hit a blind-spot wall!
    if (did_not_move && !last_was_rotate) {
        in_frontier_.insert(last_immediate_target); // Blacklist the wall
    }
    
    // --- 2. VOLUMETRIC SWEEPING ---
    // Alternate Lidar pitch to map floor and ceiling seamlessly while moving
    int tick = visited_.size();
    double pitch = 0.0;
    if (tick % 3 == 0) pitch = 30.0;
    else if (tick % 3 == 1) pitch = -30.0;
    
    Orientation dynamic_scan{
        0.0 * horizontal_angle[deg],
        pitch * altitude_angle[deg],
    };

    // --- 3. TRUE FRONTIER BFS ---
    std::unordered_set<GridCell, GridCellHash> bfs_visited;
    std::queue<GridCell> q;
    std::unordered_map<GridCell, GridCell, GridCellHash> parent;

    q.push(current);
    bfs_visited.insert(current);
    
    std::optional<GridCell> target_frontier = std::nullopt;

    while (!q.empty()) {
        GridCell c = q.front();
        q.pop();

        auto c_occ = output_map_.atVoxel(gridToWorld(c));
        
        // If the cell we pulled is Unmapped, it is our destination!
        if (c != current && c_occ == types::VoxelOccupancy::Unmapped) {
            target_frontier = c;
            break; 
        }
        
        const GridCell neighbors[6] = {
            {c.x + 1, c.y, c.z}, {c.x - 1, c.y, c.z},
            {c.x, c.y + 1, c.z}, {c.x, c.y - 1, c.z},
            {c.x, c.y, c.z + 1}, {c.x, c.y, c.z - 1}
        };
        
        for (const auto& nb : neighbors) {
            if (bfs_visited.count(nb) || in_frontier_.count(nb)) continue;

            // Enqueue only cells the drone can physically occupy: in-bounds, Empty/Unmapped, and
            // with the full spherical footprint clearing known obstacles (isNavigable). This keeps
            // the planned path width-aware — a large drone never routes through a gap too narrow
            // for its body, while a point-sized drone behaves exactly as before.
            if (isNavigable(nb)) {
                bfs_visited.insert(nb);
                parent[nb] = c;
                q.push(nb);
            }
        }
    }

    if (!target_frontier) {
        return {std::nullopt, dynamic_scan, types::AlgorithmStatus::Finished};
    }

    // --- 4. PATH EXTRACTION & COMPRESSION ---
    std::vector<GridCell> path;
    GridCell curr = *target_frontier;
    while (curr != current) {
        path.push_back(curr);
        if (parent.find(curr) == parent.end()) break;
        curr = parent[curr];
    }
    std::reverse(path.begin(), path.end());

    GridCell immediate_step = path[0];
    GridCell target_cell = immediate_step;

    // Leading-edge scan-before-enter is only ATTEMPTED when the leading edge is
    // observable: the lidar sees nothing closer than z_min, so if the drone's radius is below z_min the
    // newly-entered footprint cells sit inside that blind zone and can never be confirmed. Gating on
    // them there would stall the drone (every frontier "unconfirmable") — the large-drone regression
    // seen with z_min=20cm > radius=7.5cm. When not observable the drone enters optimistically, exactly
    // as before (collisions stay non-fatal and are mapped by the next scan).
    const double radius_cm = drone_config_.radius.numerical_value_in(cm);
    const double z_min_cm  = lidar_config_.z_min.numerical_value_in(cm);
    const bool confirm_before_enter = radius_cm > 0.0 && radius_cm >= z_min_cm;

    // Jump ahead through a straight run of already-verified-Empty cells. When confirmation is active the
    // run is capped to the per-tick move limit (whole cells within max_advance/max_elevate) so
    // target_cell is the cell the drone ACTUALLY lands on (clamps the move identically),
    // making the leading-edge check match exactly what the move enters. Otherwise the original
    // up-to-5-cell jump is kept unchanged (DroneControl still clamps the executed distance).
    if (path.size() > 1) {
        const int dx = path[0].x - current.x;
        const int dy = path[0].y - current.y;
        const int dz = path[0].z - current.z;

        std::size_t jump_limit = 5;
        if (confirm_before_enter) {
            const double max_move_cm = (dz != 0)
                ? drone_config_.max_elevate.numerical_value_in(cm)
                : drone_config_.max_advance.numerical_value_in(cm);
            const auto max_cells = static_cast<std::size_t>(
                std::max<long long>(1, static_cast<long long>(std::floor(max_move_cm / stepCm()))));
            jump_limit = std::min(jump_limit, max_cells);
        }

        for (std::size_t i = 1; i < std::min(path.size(), jump_limit); ++i) {
            if (path[i].x - path[i - 1].x == dx &&
                path[i].y - path[i - 1].y == dy &&
                path[i].z - path[i - 1].z == dz &&
                output_map_.atVoxel(gridToWorld(path[i])) == types::VoxelOccupancy::Empty) {
                target_cell = path[i];
            } else {
                break;
            }
        }
    }

    // --- 5. EXECUTE COMMAND ---
    // Leading-edge confirmation (bounded, non-stranding). When observable, a bodied drone prefers to
    // scan the voxels NEWLY entering its footprint at target_cell before committing (voxels shared with
    // the current footprint are reused). But confirmation is BEST-EFFORT: after kMaxScanAttempts scans
    // of the same cell it enters optimistically anyway rather than abandoning a reachable frontier —
    // the earlier design blacklisted here and stranded the whole drone. Unobservable leading edges
    // (radius < z_min) are never gated.
    constexpr int kMaxScanAttempts = 1;
    const bool leading_unconfirmed = confirm_before_enter &&
        !leadingFootprintClear(output_map_, gridToWorld(current), gridToWorld(target_cell),
                               drone_config_.radius);

    // Records the 3-element inter-tick state buffer (immediate target, rotate flag, current cell) the
    // stuck-detector reads next tick. rotate_flag=true marks an intentional non-move (rotate or
    // scan-before-enter hold) so a legitimate stay-in-place is not mistaken for a blind-wall stall.
    const auto pushState = [&](bool rotate_flag) {
        frontier_.push(immediate_step);
        frontier_.push(GridCell{rotate_flag ? 1 : 0, 0, 0});
        frontier_.push(current);
    };

    // Decides whether to hold and scan (instead of moving) this tick, consuming the per-cell scan
    // budget. Returns false — proceed with the move — when the leading edge is confirmed, unobservable,
    // or the budget is spent. Never blacklists: a frontier we could not confirm is entered
    // optimistically, not permanently abandoned.
    const auto takeScanHold = [&]() -> bool {
        if (!leading_unconfirmed) {
            pending_scan_cell_.reset();
            scan_attempts_ = 0;
            return false;
        }
        const bool same = pending_scan_cell_ && *pending_scan_cell_ == target_cell;
        const int attempts = same ? scan_attempts_ : 0;
        if (attempts >= kMaxScanAttempts) {
            return false; // best effort spent → enter optimistically
        }
        pending_scan_cell_ = target_cell;
        scan_attempts_ = attempts + 1;
        return true;
    };

    if (target_cell.z != current.z) {
        const double dz_cm = static_cast<double>(target_cell.z - current.z) * stepCm();
        if (takeScanHold()) {
            // The forward beams never see straight up/down, so aim vertically to reveal the cell the
            // drone is about to rise/descend into before committing the elevate.
            const double aim_pitch = (dz_cm > 0.0) ? 90.0 : -90.0;
            pushState(/*rotate_flag=*/true);
            return {std::nullopt,
                    Orientation{0.0 * horizontal_angle[deg], aim_pitch * altitude_angle[deg]},
                    types::AlgorithmStatus::Working};
        }

        const double elevate_pitch = (dz_cm > 0.0) ? 45.0 : -45.0;
        Orientation elevate_scan{0.0 * horizontal_angle[deg], elevate_pitch * altitude_angle[deg]};
        pushState(/*rotate_flag=*/false);
        return {
            types::MovementCommand{
                types::MovementCommandType::Elevate,
                types::RotationDirection::Left,
                0.0 * deg,
                dz_cm * cm,
            },
            elevate_scan,
            types::AlgorithmStatus::Working,
        };
    }

    const double cx = state.position.x.numerical_value_in(cm);
    const double cy = state.position.y.numerical_value_in(cm);
    const Position3D next_world = gridToWorld(target_cell);
    const double ndx = next_world.x.numerical_value_in(cm) - cx;
    const double ndy = next_world.y.numerical_value_in(cm) - cy;
    const double step_dist = std::sqrt(ndx * ndx + ndy * ndy);

    double desired_heading = std::atan2(ndy, ndx) * 180.0 / std::numbers::pi;
    if (desired_heading < 0.0) desired_heading += 360.0;

    const double cur_heading = state.heading.horizontal.numerical_value_in(deg);
    const double diff = normaliseDiff(desired_heading - cur_heading);

    if (std::abs(diff) > kAngleTolDeg) {
        // Rotate to face the target first; the scan along the way reveals the leading edge in that
        // direction, so by the time the drone is aligned the cells ahead are usually already confirmed.
        const types::RotationDirection dir =
            diff > 0.0 ? types::RotationDirection::Left : types::RotationDirection::Right;
        const double safe_rotate_amount = std::min(std::abs(diff), 45.0);
        pushState(/*rotate_flag=*/true);
        return {
            types::MovementCommand{
                types::MovementCommandType::Rotate,
                dir,
                safe_rotate_amount * deg,
                0.0 * cm,
            },
            dynamic_scan,
            types::AlgorithmStatus::Working,
        };
    }

    // Facing the target: best-effort confirm the leading edge before advancing (a level forward scan
    // reveals exactly the cells the move enters). Falls through to Advance when confirmed, unobservable,
    // or the scan budget is spent.
    if (takeScanHold()) {
        pushState(/*rotate_flag=*/true);
        return {std::nullopt,
                Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                types::AlgorithmStatus::Working};
    }
    pushState(/*rotate_flag=*/false);
    return {
        types::MovementCommand{
            types::MovementCommandType::Advance,
            types::RotationDirection::Left,
            0.0 * deg,
            step_dist * cm,
        },
        dynamic_scan,
        types::AlgorithmStatus::Working,
    };
}

} // namespace drone_mapper
