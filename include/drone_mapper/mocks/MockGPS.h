#pragma once

#include <drone_mapper/mocks/IGPS.h>

namespace drone_mapper {

class MockGPS final : public IGPS {
public:
    MockGPS(Position3D position, Orientation heading, PhysicalLength resolution);

    [[nodiscard]] Position3D position() const override;
    [[nodiscard]] Orientation heading() const override;

    void setPosition(Position3D position);
    void setHeading(Orientation heading);

private:
    Position3D position_{};
    Orientation heading_{};
    PhysicalLength resolution_{};
};

} // namespace drone_mapper
