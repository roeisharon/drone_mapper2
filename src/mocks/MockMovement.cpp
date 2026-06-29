#include <drone_mapper/mocks/MockMovement.h>

#include <drone_mapper/map/IMap3D.h>
#include <drone_mapper/utils/DroneFootprint.h>

#include <cmath>
#include <numbers>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

MockMovement::MockMovement(MockGPS& gps, const IMap3D& hidden_map, PhysicalLength drone_radius)
    : gps_(gps), hidden_map_(&hidden_map), drone_radius_(drone_radius) {}

bool MockMovement::destinationClear(const Position3D& dest) const {
    if (hidden_map_ == nullptr) {
        return true; // collision checking disabled (plain ctor)
    }
    // The hidden map is ground truth: only solid (Occupied) voxels block the sphere.
    return footprintFits(*hidden_map_, dest, drone_radius_,
                         [](types::VoxelOccupancy occ) {
                             return occ == types::VoxelOccupancy::Occupied;
                         });
}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    // Left rotation adds the angle; right rotation subtracts it.
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    // Wrap to [0, 360) so heading never accumulates past ±360°.
    double raw = std::fmod((current.horizontal + signed_angle).numerical_value_in(deg), 360.0);
    if (raw < 0.0) raw += 360.0;
    gps_.setHeading(Orientation{raw * deg, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D pos = gps_.position();
    // Project the distance onto the X-Y plane using the current horizontal heading:
    // heading 0° → +X, heading 90° → +Y. std::cos/std::sin take radians, so convert.
    const double heading_rad =
        gps_.heading().horizontal.numerical_value_in(deg) * std::numbers::pi / 180.0;
    const double dist_cm = distance.numerical_value_in(cm);
    const double dx = dist_cm * std::cos(heading_rad);
    const double dy = dist_cm * std::sin(heading_rad);
    const Position3D dest{
        pos.x + dx * x_extent[cm],
        pos.y + dy * y_extent[cm],
        pos.z, // advance never changes altitude
    };
    // Refuse to drive the drone's body into solid geometry; the drone stays put.
    if (!destinationClear(dest)) {
        return types::MovementResult{false, "DRONE_HITS_OBSTACLE"};
    }
    gps_.setPosition(dest);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Purely vertical: a negative distance descends. No clamping here — clamping is
    // DroneControlImpl's responsibility.
    const Position3D pos = gps_.position();
    const Position3D dest{
        pos.x,
        pos.y,
        pos.z + distance.numerical_value_in(cm) * z_extent[cm],
    };
    // Refuse to drive the drone's body into a floor/ceiling/solid; the drone stays put.
    if (!destinationClear(dest)) {
        return types::MovementResult{false, "DRONE_HITS_OBSTACLE"};
    }
    gps_.setPosition(dest);
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
