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
    // Navigable only if INSIDE the map and not a known obstacle. Unmapped means not yet
    // scanned — treated as passable (optimistic) so exploration can proceed. OutOfBounds must
    // NOT be navigable: atVoxel() returns OutOfBounds (not Occupied) past the map edge, so
    // without this the frontier leaks into the infinite empty space outside the map and the
    // algorithm never returns Finished. PotentiallyOccupied is an obstacle too: the drone must
    // not navigate into a too-close, uncertain detection (collision safety).
    const Position3D center = gridToWorld(cell);
    const types::VoxelOccupancy v = output_map_.atVoxel(center);
    if (v == types::VoxelOccupancy::Occupied ||
        v == types::VoxelOccupancy::PotentiallyOccupied ||
        v == types::VoxelOccupancy::OutOfBounds) {
        return false;
    }
    // The cell itself is free, but the drone is a sphere of drone_config_.radius — it can only
    // occupy this cell if its full footprint clears every known obstacle. This stops the planner
    // routing a large drone through gaps a point-sized drone could slip through. With a zero
    // radius (default config) this reduces to the single-cell test above.
    return footprintFits(output_map_, center, drone_config_.radius,
                         [](types::VoxelOccupancy occ) {
                             return occ == types::VoxelOccupancy::Occupied ||
                                    occ == types::VoxelOccupancy::PotentiallyOccupied;
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

    // Jump up to 50cm if the hallway ahead is verified empty
    if (path.size() > 1) {
        int dx = path[0].x - current.x;
        int dy = path[0].y - current.y;
        int dz = path[0].z - current.z;

        for (size_t i = 1; i < std::min(path.size(), (size_t)5); ++i) {
            if (path[i].x - path[i-1].x == dx &&
                path[i].y - path[i-1].y == dy &&
                path[i].z - path[i-1].z == dz) {

                if (output_map_.atVoxel(gridToWorld(path[i])) == types::VoxelOccupancy::Empty) {
                    target_cell = path[i];
                } else {
                    break;
                }
            } else {
                break;
            }
        }
    }

    // --- 5. EXECUTE COMMAND ---
    if (target_cell.z != current.z) {
        const double dz_cm = static_cast<double>(target_cell.z - current.z) * stepCm();
        
        // Pitch lidar in the direction of elevation
        double elevate_pitch = (dz_cm > 0) ? 45.0 : -45.0;
        Orientation elevate_scan{0.0 * horizontal_angle[deg], elevate_pitch * altitude_angle[deg]};
        
        frontier_.push(immediate_step); // Store immediate block for accurate blacklisting
        frontier_.push({0, 0, 0});      // Rotate flag = false
        frontier_.push(current);

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
        const types::RotationDirection dir = diff > 0.0 ? types::RotationDirection::Left : types::RotationDirection::Right;
        double safe_rotate_amount = std::min(std::abs(diff), 45.0);
        
        frontier_.push(immediate_step);
        frontier_.push({1, 0, 0}); // Rotate flag = true
        frontier_.push(current);

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

    frontier_.push(immediate_step);
    frontier_.push({0, 0, 0}); // Rotate flag = false
    frontier_.push(current);

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
