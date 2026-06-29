#pragma once

#include <drone_mapper/mocks/IDroneMovement.h>
#include <drone_mapper/mocks/MockGPS.h>

namespace drone_mapper {

class IMap3D;

// Optional implementation for the
class MockMovement final : public IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps);

    // Collision-aware overload: advance() and elevate() validate the destination against the hidden
    // ground-truth map. A move is refused (no position change, MovementResult{false, ...}) when the
    // drone's spherical footprint (drone_radius) would intersect an Occupied voxel. This is how drone
    // size limits where the drone can physically go. The plain ctor above leaves collision checking
    // off (footprint never tested), preserving the original always-succeeds behaviour.
    MockMovement(MockGPS& gps, const IMap3D& hidden_map, PhysicalLength drone_radius);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    // Returns true if the drone's footprint fits at dest in the hidden map (always true when no
    // hidden map was injected). Shared by advance() and elevate().
    [[nodiscard]] bool destinationClear(const Position3D& dest) const;

    MockGPS& gps_;
    const IMap3D* hidden_map_ = nullptr; // null → collision checking disabled
    PhysicalLength drone_radius_{};
};

} // namespace drone_mapper
