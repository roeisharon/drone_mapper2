#include <drone_mapper/utils/MapsComparison.h>
#include <drone_mapper/Units.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace drone_mapper {

namespace {

// Returns true if pos has at least one Empty neighbor in origin along the six cardinal axes.
// LiDAR beams stop at the first Occupied voxel they hit, so an Occupied voxel is only
// observable when a beam can reach it from an adjacent Empty voxel.
bool hasEmptyNeighbor(const IMap3D& origin, const Position3D& pos, double res_cm) {
    const std::array<Position3D, 6> neighbors{{
        {pos.x + res_cm * x_extent[cm], pos.y, pos.z},
        {pos.x - res_cm * x_extent[cm], pos.y, pos.z},
        {pos.x, pos.y + res_cm * y_extent[cm], pos.z},
        {pos.x, pos.y - res_cm * y_extent[cm], pos.z},
        {pos.x, pos.y, pos.z + res_cm * z_extent[cm]},
        {pos.x, pos.y, pos.z - res_cm * z_extent[cm]},
    }};
    for (const auto& n : neighbors) {
        if (origin.atVoxel(n) == types::VoxelOccupancy::Empty) {
            return true;
        }
    }
    return false;
}

} // namespace

// Scores each target map against the authoritative origin (ground-truth) map.
//
// Algorithm:
//   1. Read origin.getMapConfig() for world-coordinate bounds and resolution.
//   2. Guard: non-positive resolution → return all-zero scores (cannot iterate a grid).
//   3. Derive integer voxel counts per axis from (max - min) / resolution.
//   4. For every voxel position in the origin's extent:
//        a. origin_val = origin.atVoxel(pos).
//        b. Skip Unmapped / OutOfBounds — not ground truth; excluded from scoring.
//        c. For Occupied voxels, skip interior voxels that have no Empty neighbor —
//           a LiDAR beam can never reach them, so they are unobservable and must not
//           penalise the drone's score.
//        d. Otherwise count the voxel as mappable; each target matches iff its value at
//           the same world position equals origin_val exactly.
//   5. score[t] = 100 * matching[t] / total_mappable   (0 when total_mappable == 0).
//
// PotentiallyOccupied policy: scoring is exact-match. The origin map is loaded ground truth
// and only ever holds Empty/Occupied/Unmapped; PotentiallyOccupied (-3) is produced solely by
// the output map. A target PotentiallyOccupied therefore only scores when the origin voxel is
// also PotentiallyOccupied — at an origin Empty/Occupied cell it counts as a miss.
//
// The target loop is nested inside the voxel triple-loop so each origin voxel is read once
// regardless of target count: O(n_voxels) origin lookups, O(n_voxels * n_targets) total.
std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    if (targets.empty()) {
        return {};
    }

    const types::MapConfig config = origin.getMapConfig();
    const double res_cm = config.resolution.numerical_value_in(cm);

    if (res_cm <= 0.0) {
        return std::vector<double>(targets.size(), 0.0);
    }

    const double min_x_cm = config.boundaries.min_x.numerical_value_in(cm);
    const double max_x_cm = config.boundaries.max_x.numerical_value_in(cm);
    const double min_y_cm = config.boundaries.min_y.numerical_value_in(cm);
    const double max_y_cm = config.boundaries.max_y.numerical_value_in(cm);
    const double min_z_cm = config.boundaries.min_height.numerical_value_in(cm);
    const double max_z_cm = config.boundaries.max_height.numerical_value_in(cm);

    // std::round absorbs floating-point error in the division; std::max(0.0, ...) guards
    // against inverted or empty ranges (a degenerate config yields zero voxels, score 0).
    const auto n_x = static_cast<std::size_t>(
        std::max(0.0, std::round((max_x_cm - min_x_cm) / res_cm)));
    const auto n_y = static_cast<std::size_t>(
        std::max(0.0, std::round((max_y_cm - min_y_cm) / res_cm)));
    const auto n_z = static_cast<std::size_t>(
        std::max(0.0, std::round((max_z_cm - min_z_cm) / res_cm)));

    const std::size_t n_targets = targets.size();
    std::vector<std::size_t> matching(n_targets, 0);
    std::size_t total_mappable = 0;

    for (std::size_t ix = 0; ix < n_x; ++ix) {
        const double x_cm = min_x_cm + static_cast<double>(ix) * res_cm;

        for (std::size_t iy = 0; iy < n_y; ++iy) {
            const double y_cm = min_y_cm + static_cast<double>(iy) * res_cm;

            for (std::size_t iz = 0; iz < n_z; ++iz) {
                const double z_cm = min_z_cm + static_cast<double>(iz) * res_cm;

                const Position3D pos{
                    x_cm * x_extent[cm],
                    y_cm * y_extent[cm],
                    z_cm * z_extent[cm]
                };

                const auto origin_val = origin.atVoxel(pos);

                if (origin_val == types::VoxelOccupancy::Unmapped ||
                    origin_val == types::VoxelOccupancy::OutOfBounds) {
                    continue;
                }

                // Interior Occupied voxels (no Empty neighbor) are unreachable by any
                // LiDAR beam and must not count against the drone's mapping score.
                if (origin_val == types::VoxelOccupancy::Occupied &&
                    !hasEmptyNeighbor(origin, pos, res_cm)) {
                    continue;
                }

                ++total_mappable;

                for (std::size_t t = 0; t < n_targets; ++t) {
                    if (targets[t]->atVoxel(pos) == origin_val) {
                        ++matching[t];
                    }
                }
            }
        }
    }

    std::vector<double> scores(n_targets);
    for (std::size_t t = 0; t < n_targets; ++t) {
        scores[t] = (total_mappable == 0)
            ? 0.0
            : 100.0 * static_cast<double>(matching[t])
                    / static_cast<double>(total_mappable);
    }
    return scores;
}

} // namespace drone_mapper
