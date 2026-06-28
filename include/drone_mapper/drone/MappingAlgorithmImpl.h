#pragma once

#include <drone_mapper/drone/IMappingAlgorithm.h>

#include <cstddef>
#include <optional>
#include <queue>
#include <unordered_set>

namespace drone_mapper {

// BFS-based 3-D mapping algorithm.
//
// Exploration model:
//   - The internal grid is quantized at steps equal to the output map's resolution.
//   - Frontier targets are Unmapped cells (Occupied, PotentiallyOccupied, and OutOfBounds are
//     blocked and never selected); Unmapped neighbours are enqueued optimistically for exploration.
//   - Scan-before-enter: the drone only physically MOVES into a cell already confirmed Empty in the
//     output map. If the next cell toward the target is still Unmapped, the algorithm points the
//     lidar at it (forward for a horizontal step, straight up/down for a vertical step) and scans
//     WITHOUT moving, so the next call sees it as Empty (enter) or an obstacle (route around). This
//     prevents the drone flying into unscanned walls — essential for vertical moves, since the
//     forward scan never reveals the cells directly above/below.
//   - nextStep() is a reactive state machine: each call inspects the current DroneState and the live
//     output map, then returns a MappingStepCommand.
//   - When the BFS frontier is exhausted, AlgorithmStatus::Finished is returned.
//   - A scan is requested on every working step so the map is built up continuously.
class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    // Inherit the base-class constructor (mission, lidar, drone configs + output map).
    using IMappingAlgorithm::IMappingAlgorithm;

    // Returns the next MappingStepCommand. latest_scan is nullptr on the first call and may
    // be nullptr on any step; the output map (output_map_) is the authoritative source.
    [[nodiscard]] types::MappingStepCommand nextStep(
        const types::DroneState& state,
        const types::LidarScanResult* latest_scan) override;

private:
    // Integer-quantized 3-D grid coordinate; one unit equals the output map's resolution.
    struct GridCell {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const GridCell& o) const noexcept {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct GridCellHash {
        std::size_t operator()(const GridCell& c) const noexcept;
    };

    using CellSet   = std::unordered_set<GridCell, GridCellHash>;
    using CellQueue = std::queue<GridCell>;

    // Exploration state.
    CellSet   visited_;       // cells the drone has physically occupied
    CellSet   in_frontier_;   // dedup guard: cells currently queued
    CellQueue frontier_;      // BFS frontier (FIFO)
    std::optional<GridCell> current_target_; // cell currently being navigated toward

    // Grid step size (cm) from the output map's resolution.
    [[nodiscard]] double stepCm() const noexcept;
    // World position -> nearest grid cell.
    [[nodiscard]] GridCell worldToGrid(const Position3D& pos) const noexcept;
    // Grid cell -> its world-space centre position.
    [[nodiscard]] Position3D gridToWorld(const GridCell& cell) const noexcept;
    // Enqueue all 6-connected neighbours that are unvisited, not already queued, and navigable.
    //void expandNeighbors(const GridCell& cell);
    // True iff the cell is in-bounds and not a known obstacle (Occupied/PotentiallyOccupied).
    [[nodiscard]] bool isNavigable(const GridCell& cell) const noexcept;
};

} // namespace drone_mapper
