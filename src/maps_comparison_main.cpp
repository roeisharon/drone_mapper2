#include <drone_mapper/utils/MapsComparison.h>
#include <drone_mapper/utils/YamlConfigParser.h>
#include <drone_mapper/utils/ErrorLogger.h>
#include <drone_mapper/map/Map3DImpl.h>
#include <drone_mapper/Units.h>

#include <TinyNPY.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

// Load a .npy file into an NpyArray; throws std::runtime_error on failure.
std::shared_ptr<NpyArray> loadNpy(const std::filesystem::path& path) {
    auto arr = std::make_shared<NpyArray>();
    const std::string path_str = path.string();
    const char* error = arr->LoadNPY(path_str.c_str());
    if (error != nullptr) {
        throw std::runtime_error("Failed to load '" + path_str + "': " + error);
    }
    return arr;
}

// Build a Map3DImpl from a loaded NpyArray at the given resolution with no offset. Map3DImpl derives
// world boundaries from the array shape (min = 0, max = shape * res).
drone_mapper::Map3DImpl makeMap(std::shared_ptr<NpyArray> npy, double res_cm) {
    const drone_mapper::types::MapConfig cfg{
        drone_mapper::types::MappingBounds{}, drone_mapper::Position3D{}, res_cm * drone_mapper::cm};
    return drone_mapper::Map3DImpl{std::move(npy), cfg};
}

// BONUS: build a Map3DImpl from a comparison_config side (its resolution + offset). A non-positive
// resolution falls back to 1 cm. map_boundaries are parsed but not applied: the frozen 2-arg
// compare() iterates the origin's full extent, so a sub-region would need a wider compare API.
drone_mapper::Map3DImpl makeMapFromSpec(std::shared_ptr<NpyArray> npy,
                                        const drone_mapper::config::MapComparisonSpec& spec) {
    const double res_cm = spec.map_resolution.numerical_value_in(drone_mapper::cm);
    const drone_mapper::types::MapConfig cfg{
        drone_mapper::types::MappingBounds{}, spec.map_offset,
        (res_cm > 0.0 ? res_cm : 1.0) * drone_mapper::cm};
    return drone_mapper::Map3DImpl{std::move(npy), cfg};
}

} // namespace

// maps_comparison
//
// Usage:
//   ./maps_comparison <origin_map.npy> <target_map.npy> [comparison_config=<path>]
//
// Prints the similarity score (0-100) for the target against the origin to stdout.
// On any error: prints -1 to stdout and a descriptive message to stderr.
//
// Base (no comparison_config): both maps load at 1 cm/voxel with no offset, sharing the same
// shape/resolution/offset (Assignment-1 assumption); compare() iterates the origin's full extent.
// BONUS: comparison_config=<path> applies each side's resolution and offset; map_boundaries are
// parsed but not applied (the frozen compare() has no sub-region parameter).
int main(int argc, char** argv) {
    try {
        if (argc < 3 || argc > 4) {
            std::cerr << "Usage: maps_comparison <origin_map.npy> <target_map.npy>"
                         " [comparison_config=<path>]\n";
            std::cout << -1 << "\n";
            return 1;
        }

        const std::filesystem::path origin_path{argv[1]};
        const std::filesystem::path target_path{argv[2]};

        // BONUS: detect and parse the optional comparison_config argument.
        std::optional<drone_mapper::config::ComparisonConfig> comparison_config;
        if (argc == 4) {
            const std::string arg3{argv[3]};
            const std::string prefix = "comparison_config=";
            if (arg3.rfind(prefix, 0) != 0) {
                std::cerr << "Unknown argument: " << arg3 << "\n";
                std::cout << -1 << "\n";
                return 1;
            }
            const std::filesystem::path config_path{arg3.substr(prefix.size())};
            drone_mapper::ErrorLogger logger{
                std::filesystem::temp_directory_path() / "maps_comparison_errors.log"};
            comparison_config = drone_mapper::config::parseComparisonConfig(config_path, logger);
            if (!comparison_config) {
                std::cerr << "Warning: could not parse comparison_config '" << config_path.string()
                          << "'; falling back to shared resolution, offset, and boundaries.\n";
            }
        }

        const auto origin_npy = loadNpy(origin_path);
        const auto target_npy = loadNpy(target_path);

        constexpr double kDefaultResCm = 1.0;
        auto origin_map = comparison_config ? makeMapFromSpec(origin_npy, comparison_config->original)
                                            : makeMap(origin_npy, kDefaultResCm);
        auto target_map = comparison_config ? makeMapFromSpec(target_npy, comparison_config->target)
                                            : makeMap(target_npy, kDefaultResCm);

        const auto scores = drone_mapper::MapsComparison::compare(origin_map, {&target_map});

        if (scores.empty()) {
            std::cerr << "Error: comparison returned no scores\n";
            std::cout << -1 << "\n";
            return 0;
        }

        std::cout << scores[0] << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cout << -1 << "\n";
        return 0;
    }
}
