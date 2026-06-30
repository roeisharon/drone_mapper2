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

// World-coordinate intersection of two boundary boxes.
types::MappingBounds intersect(const types::MappingBounds& a, const types::MappingBounds& b) {
    return types::MappingBounds{
        std::max(a.min_x, b.min_x),           std::min(a.max_x, b.max_x),
        std::max(a.min_y, b.min_y),           std::min(a.max_y, b.max_y),
        std::max(a.min_height, b.min_height), std::min(a.max_height, b.max_height),
    };
}

// Scores one target against origin over a world-coordinate region, at the given resolution.
// Iterates the region's voxels (cell centres, for unambiguous quantisation) and, for every mappable
// origin voxel (Empty, or an Occupied surface voxel — interior Occupied voxels are excluded as
// unobservable), counts the target as matching iff it reports the same value at that world position.
// Querying by world position means origin and target may differ in shape/offset; only their overlap
// is scored, so a mission-bounded output map is judged only over the region it was asked to map.
double scoreTarget(const IMap3D& origin, const IMap3D& target,
                   const types::MappingBounds& region, double res_cm) {
    const double min_x = region.min_x.numerical_value_in(cm);
    const double max_x = region.max_x.numerical_value_in(cm);
    const double min_y = region.min_y.numerical_value_in(cm);
    const double max_y = region.max_y.numerical_value_in(cm);
    const double min_z = region.min_height.numerical_value_in(cm);
    const double max_z = region.max_height.numerical_value_in(cm);

    const auto n_x = static_cast<std::size_t>(std::max(0.0, std::round((max_x - min_x) / res_cm)));
    const auto n_y = static_cast<std::size_t>(std::max(0.0, std::round((max_y - min_y) / res_cm)));
    const auto n_z = static_cast<std::size_t>(std::max(0.0, std::round((max_z - min_z) / res_cm)));

    std::size_t total = 0;
    std::size_t matching = 0;

    for (std::size_t ix = 0; ix < n_x; ++ix) {
        const double x_cm = min_x + (static_cast<double>(ix) + 0.5) * res_cm;
        for (std::size_t iy = 0; iy < n_y; ++iy) {
            const double y_cm = min_y + (static_cast<double>(iy) + 0.5) * res_cm;
            for (std::size_t iz = 0; iz < n_z; ++iz) {
                const double z_cm = min_z + (static_cast<double>(iz) + 0.5) * res_cm;
                const Position3D pos{x_cm * x_extent[cm], y_cm * y_extent[cm], z_cm * z_extent[cm]};

                const auto origin_val = origin.atVoxel(pos);
                if (origin_val == types::VoxelOccupancy::Unmapped ||
                    origin_val == types::VoxelOccupancy::OutOfBounds) {
                    continue;
                }
                if (origin_val == types::VoxelOccupancy::Occupied &&
                    !hasEmptyNeighbor(origin, pos, res_cm)) {
                    continue; // unreachable interior solid — unobservable, not counted
                }
                ++total;
                if (target.atVoxel(pos) == origin_val) {
                    ++matching;
                }
            }
        }
    }
    return (total == 0) ? 0.0 : 100.0 * static_cast<double>(matching) / static_cast<double>(total);
}

} // namespace

// Scores each target map against the authoritative origin (ground-truth) map.
//
// Each target is scored ONLY over the world region it represents: the intersection of the origin's
// extent with that target's extent. This matters when a mission-bounded output map covers a
// sub-region of the hidden map — voxels outside the output region are not the drone's responsibility
// and must not be counted as misses. When the target covers the whole origin (the common/benchmark
// case) the region is the origin's full extent, so scoring is unchanged.
//
// Comparison is by WORLD coordinate (not array index), so origin and target may have different
// shapes/offsets. Per voxel: skip origin Unmapped/OutOfBounds (not ground truth) and interior
// Occupied voxels with no Empty neighbour (unobservable); otherwise count it mappable and a match
// iff the target reports the same value at that world position.
//
// PotentiallyOccupied policy: exact-match. The origin is loaded ground truth (Empty/Occupied only);
// a target PotentiallyOccupied at an origin Empty/Occupied cell is a miss.
std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    if (targets.empty()) {
        return {};
    }

    const types::MapConfig origin_config = origin.getMapConfig();
    const double res_cm = origin_config.resolution.numerical_value_in(cm);
    if (res_cm <= 0.0) {
        return std::vector<double>(targets.size(), 0.0);
    }

    std::vector<double> scores(targets.size(), 0.0);
    for (std::size_t t = 0; t < targets.size(); ++t) {
        const types::MappingBounds region =
            intersect(origin_config.boundaries, targets[t]->getMapConfig().boundaries);
        scores[t] = scoreTarget(origin, *targets[t], region, res_cm);
    }
    return scores;
}

} // namespace drone_mapper
