#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class ISimulationRun {
public:
    virtual ~ISimulationRun() = default;

    // Changed: a run now returns simulation-level data, including score and output-map metadata.
    [[nodiscard]] virtual types::SimulationResult run() = 0;
};

} // namespace drone_mapper
