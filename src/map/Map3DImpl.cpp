#include <drone_mapper/map/Map3DImpl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

namespace {

// True when integer indices (x, y, z) all fall within [0, size) on their axis.
bool inBounds(long long x, long long y, long long z,
              std::size_t xs, std::size_t ys, std::size_t zs) noexcept {
    return x >= 0 && y >= 0 && z >= 0 &&
           static_cast<std::size_t>(x) < xs &&
           static_cast<std::size_t>(y) < ys &&
           static_cast<std::size_t>(z) < zs;
}

// Row-major (C order) flattening: flat = x*ys*zs + y*zs + z.
std::size_t flatIndex(std::size_t x, std::size_t y, std::size_t z,
                      std::size_t ys, std::size_t zs) noexcept {
    return x * ys * zs + y * zs + z;
}

// World coordinate -> zero-based array index on one axis: floor((world + offset) / res).
// The offset shifts the logical origin, so world (0,0,0) maps to index (offset/res);
// negative world coordinates are valid when the offset places the origin inside the array.
long long toIndex(double world_cm, double offset_cm, double res_cm) noexcept {
    return static_cast<long long>(std::floor((world_cm + offset_cm) / res_cm));
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)), config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl: map pointer must not be null.");
    }

    if (!map_->IsEmpty()) {
        // Path A: loaded from file — derive dimensions from the array shape and copy the data.
        const auto& shape = map_->Shape();
        if (shape.size() != 3) {
            throw std::invalid_argument("Map3DImpl: NPY array must be 3-dimensional.");
        }
        if (map_->SizeValueBytes() != 1) {
            throw std::invalid_argument("Map3DImpl: only 1-byte-per-voxel NPY arrays are supported.");
        }

        x_size_ = shape[0];
        y_size_ = shape[1];
        z_size_ = shape[2];

        // Recompute boundaries from shape + offset + resolution so getMapConfig() always
        // reflects the real spatial extent (the factory passes zeroed MappingBounds here).
        //   world at ix=0 -> -offset ; world at ix=N -> N*res - offset (exclusive upper bound).
        const double res = config_.resolution.numerical_value_in(cm);
        const double ox  = config_.offset.x.numerical_value_in(cm);
        const double oy  = config_.offset.y.numerical_value_in(cm);
        const double oz  = config_.offset.z.numerical_value_in(cm);
        config_.boundaries = {
            -ox                                        * x_extent[cm],
            (static_cast<double>(x_size_) * res - ox)  * x_extent[cm],
            -oy                                        * y_extent[cm],
            (static_cast<double>(y_size_) * res - oy)  * y_extent[cm],
            -oz                                        * z_extent[cm],
            (static_cast<double>(z_size_) * res - oz)  * z_extent[cm],
        };

        // Byte-level copy into int8. uint8 source files (0=Empty, 1=Occupied) and int8 source
        // files (negative Unmapped/PotentiallyOccupied) both copy correctly under two's complement.
        const std::size_t total = x_size_ * y_size_ * z_size_;
        data_.resize(total);
        const auto* raw = map_->Data<std::uint8_t>();
        for (std::size_t i = 0; i < total; ++i) {
            //data_[i] = static_cast<std::int8_t>(raw[i]);
            // Turn any non-zero value into Occupied (1) for the output map, so that any Minecraft-style block ID is treated as a solid block.
            const std::uint8_t v = raw[i];
            // Negative enum states are stored two's-complement in int8: -1=255 (Unmapped),
            // -2=254 (OutOfBounds), -3=253 (PotentiallyOccupied). Decode them back to the enum
            // byte; 0 stays Empty; every other (positive Minecraft block id) becomes Occupied.
            if (v == 0) {
                data_[i] = static_cast<std::int8_t>(types::VoxelOccupancy::Empty);
            } else if (v == 255) {
                data_[i] = static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped);
            } else if (v == 254) {
                data_[i] = static_cast<std::int8_t>(types::VoxelOccupancy::OutOfBounds);
            } else if (v == 253) {
                data_[i] = static_cast<std::int8_t>(types::VoxelOccupancy::PotentiallyOccupied);
            } else {
                data_[i] = static_cast<std::int8_t>(types::VoxelOccupancy::Occupied);
            }
            
        }
    } else {
        // Path B: empty array (output map) — size storage from the config boundaries.
        const double res = config_.resolution.numerical_value_in(cm);
        if (res > 0.0) {
            const auto& b = config_.boundaries;
            const double dx = (b.max_x      - b.min_x     ).numerical_value_in(cm);
            const double dy = (b.max_y      - b.min_y     ).numerical_value_in(cm);
            const double dz = (b.max_height - b.min_height).numerical_value_in(cm);
            x_size_ = static_cast<std::size_t>(std::max(0.0, dx / res));
            y_size_ = static_cast<std::size_t>(std::max(0.0, dy / res));
            z_size_ = static_cast<std::size_t>(std::max(0.0, dz / res));
        }
        // Every voxel starts Unmapped; set() fills them in as the drone scans.
        data_.assign(x_size_ * y_size_ * z_size_,
                     static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    }
}

std::optional<std::size_t> Map3DImpl::flatIndexFor(const Position3D& pos) const {
    const double res = config_.resolution.numerical_value_in(cm);
    if (res <= 0.0) {
        return std::nullopt;
    }
    const long long ix = toIndex(pos.x.numerical_value_in(cm), config_.offset.x.numerical_value_in(cm), res);
    const long long iy = toIndex(pos.y.numerical_value_in(cm), config_.offset.y.numerical_value_in(cm), res);
    const long long iz = toIndex(pos.z.numerical_value_in(cm), config_.offset.z.numerical_value_in(cm), res);
    if (!inBounds(ix, iy, iz, x_size_, y_size_, z_size_)) {
        return std::nullopt; // record as nullopt when out of bounds, so atVoxel and set can handle it consistently.
    }
    return flatIndex(static_cast<std::size_t>(ix), static_cast<std::size_t>(iy),
                     static_cast<std::size_t>(iz), y_size_, z_size_);
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    const std::optional<std::size_t> idx = flatIndexFor(pos);
    if (!idx) {
        return types::VoxelOccupancy::OutOfBounds;
    }
    // Decode the stored byte. Input .npy maps are Minecraft-style: 0 = air (Empty), any other block
    // id = a solid block (Occupied). Output maps store the VoxelOccupancy enum directly, including the
    // negative Unmapped/PotentiallyOccupied states. So 0 and the three negative enum values map to
    // themselves; every other (positive block id) normalizes to Occupied.
    const std::int8_t b = data_[*idx];
    switch (b) {
        case static_cast<std::int8_t>(types::VoxelOccupancy::Empty):               return types::VoxelOccupancy::Empty;
        case static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped):            return types::VoxelOccupancy::Unmapped;
        case static_cast<std::int8_t>(types::VoxelOccupancy::OutOfBounds):         return types::VoxelOccupancy::OutOfBounds;
        case static_cast<std::int8_t>(types::VoxelOccupancy::PotentiallyOccupied): return types::VoxelOccupancy::PotentiallyOccupied;
        default:                                                                   return types::VoxelOccupancy::Occupied;
    }
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    // Consistent with atVoxel by construction: both ask flatIndexFor, so in-bounds here
    // is exactly the set of positions atVoxel does not report OutOfBounds for.
    return flatIndexFor(pos).has_value();
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    // Out-of-bounds writes are silently ignored: scan rays may land fractionally outside
    // the map near its boundary, and dropping those is correct.
    const std::optional<std::size_t> idx = flatIndexFor(pos);
    if (idx) {
        data_[*idx] = static_cast<std::int8_t>(value);
    }
}

void Map3DImpl::save(const std::filesystem::path& output_path) const {
    if (x_size_ == 0 || y_size_ == 0 || z_size_ == 0) {
        throw std::runtime_error("Map3DImpl::save: cannot save a zero-size map.");
    }
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }

    // Write the .npy v1.0 format manually rather than via TinyNPY's SaveNPY template, which can
    // mis-handle ownership of the data pointer for signed integer types (double-free risk).
    // Layout: magic(6) + major(1) + minor(1) + header_len(2, LE) + header + raw data,
    // where the 10-byte prefix plus header_len is padded to a multiple of 64 bytes.
    std::ostringstream dict;
    dict << "{'descr': '<i1', 'fortran_order': False, 'shape': ("
         << x_size_ << ", " << y_size_ << ", " << z_size_ << "), }";

    constexpr std::size_t kPrefixLen = 10;
    std::size_t dict_len = dict.str().size() + 1; // +1 for the trailing '\n'
    const std::size_t total = kPrefixLen + dict_len;
    const std::size_t padded = ((total + 63) / 64) * 64;
    dict_len = padded - kPrefixLen;

    std::string header = dict.str();
    header.resize(dict_len - 1, ' '); // pad with spaces to the alignment boundary
    header += '\n';

    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Map3DImpl::save: cannot open output file: " + output_path.string());
    }
    out.write("\x93NUMPY", 6);
    out.put('\x01');
    out.put('\x00');
    const auto hlen = static_cast<std::uint16_t>(dict_len);
    out.write(reinterpret_cast<const char*>(&hlen), 2); // little-endian on supported platforms
    out.write(header.data(), static_cast<std::streamsize>(dict_len));
    out.write(reinterpret_cast<const char*>(data_.data()), static_cast<std::streamsize>(data_.size()));
    if (!out) {
        throw std::runtime_error("Map3DImpl::save: write failed: " + output_path.string());
    }
}

} // namespace drone_mapper
