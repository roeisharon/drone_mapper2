#include <drone_mapper/mocks/MockMovement.h>

#include <cmath>
#include <numbers>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

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
    gps_.setPosition(Position3D{
        pos.x + dx * x_extent[cm],
        pos.y + dy * y_extent[cm],
        pos.z, // advance never changes altitude
    });
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Purely vertical: a negative distance descends. No clamping here — clamping is
    // DroneControlImpl's responsibility.
    const Position3D pos = gps_.position();
    gps_.setPosition(Position3D{
        pos.x,
        pos.y,
        pos.z + distance.numerical_value_in(cm) * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
