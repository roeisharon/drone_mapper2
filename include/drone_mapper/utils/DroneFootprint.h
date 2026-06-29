#pragma once

#include <drone_mapper/map/IMap3D.h>
#include <drone_mapper/Units.h>

#include <cmath>

namespace drone_mapper {

// Tests whether the drone — modelled as a sphere of `radius` centred at world position `center` —
// physically fits without intersecting any voxel of `map` for which `is_blocking(occupancy)` is true.
//
// Used in two places so the drone is never treated as a dimensionless point:
//   - path planning  (against the output map): block on Occupied / PotentiallyOccupied;
//   - movement legality (against the hidden ground-truth map): block on Occupied.
//
// A voxel only blocks when the sphere strictly penetrates its cell cube
// (distance(center, voxel-AABB) < radius). A footprint that merely *touches* a wall face still fits,
// so a drone of diameter D exactly fits an opening of width D (e.g. a 1 cm-diameter drone fits a
// 1-voxel / 1 cm corridor, while a 2 cm-diameter drone does not). A point drone (radius 0) reduces
// to a single-cell test, preserving the original point-sized behaviour.
template <typename BlockingPred>
[[nodiscard]] bool footprintFits(const IMap3D& map,
                                 const Position3D& center,
                                 PhysicalLength radius,
                                 BlockingPred is_blocking) {
    const types::MapConfig cfg = map.getMapConfig();
    const double res = cfg.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        return true; // degenerate map: nothing meaningful to test against
    }

    const double r = radius.numerical_value_in(cm);
    if (r <= 0.0) {
        return !is_blocking(map.atVoxel(center)); // point drone: only its own voxel matters
    }

    const double cx = center.x.numerical_value_in(cm);
    const double cy = center.y.numerical_value_in(cm);
    const double cz = center.z.numerical_value_in(cm);
    const double ox = cfg.offset.x.numerical_value_in(cm);
    const double oy = cfg.offset.y.numerical_value_in(cm);
    const double oz = cfg.offset.z.numerical_value_in(cm);

    // World coordinate -> array index uses floor((world + offset) / res), matching Map3DImpl. The
    // sphere can only reach cells whose index lies within the [center - r, center + r] box.
    const auto cellIndex = [res](double world, double off) {
        return static_cast<long long>(std::floor((world + off) / res));
    };
    const long long lo_x = cellIndex(cx - r, ox), hi_x = cellIndex(cx + r, ox);
    const long long lo_y = cellIndex(cy - r, oy), hi_y = cellIndex(cy + r, oy);
    const long long lo_z = cellIndex(cz - r, oz), hi_z = cellIndex(cz + r, oz);

    // Strict-penetration threshold; the tiny epsilon keeps an exactly-touching face from counting.
    constexpr double kEpsCm = 1e-6;
    const double thresh = r - kEpsCm;
    const double thresh_sq = thresh * thresh;

    // Distance from a coordinate to a cell's [lo, hi] span on one axis (0 when inside).
    const auto axisGap = [](double c, double lo, double hi) {
        if (c < lo) return lo - c;
        if (c > hi) return c - hi;
        return 0.0;
    };

    for (long long ix = lo_x; ix <= hi_x; ++ix) {
        const double cell_lo_x = static_cast<double>(ix) * res - ox;
        const double dx = axisGap(cx, cell_lo_x, cell_lo_x + res);
        for (long long iy = lo_y; iy <= hi_y; ++iy) {
            const double cell_lo_y = static_cast<double>(iy) * res - oy;
            const double dy = axisGap(cy, cell_lo_y, cell_lo_y + res);
            for (long long iz = lo_z; iz <= hi_z; ++iz) {
                const double cell_lo_z = static_cast<double>(iz) * res - oz;
                const double dz = axisGap(cz, cell_lo_z, cell_lo_z + res);

                if (dx * dx + dy * dy + dz * dz >= thresh_sq) {
                    continue; // this cell lies entirely outside the sphere
                }
                // Sample the cell centre so atVoxel resolves to exactly (ix, iy, iz).
                const Position3D sample{
                    (cell_lo_x + 0.5 * res) * x_extent[cm],
                    (cell_lo_y + 0.5 * res) * y_extent[cm],
                    (cell_lo_z + 0.5 * res) * z_extent[cm],
                };
                if (is_blocking(map.atVoxel(sample))) {
                    return false;
                }
            }
        }
    }
    return true;
}

} // namespace drone_mapper
