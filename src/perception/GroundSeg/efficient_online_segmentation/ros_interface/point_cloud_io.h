#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ground_seg
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

// Decode XYZ for classification, while keeping original records for lossless selection.
// Row padding, arbitrary field order and both byte orders are supported.
struct CloudView
{
  explicit CloudView(const Cloud& cloud) : cloud(cloud)
  {
    for (std::size_t i = 0; i < 3; ++i) {
      auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(), [&](const Field& f) {
        return f.name == std::array<const char*, 3>{{"x", "y", "z"}}[i];
      });
      if (it == cloud.fields.end() || it->count != 1 ||
          (it->datatype != Field::FLOAT32 && it->datatype != Field::FLOAT64))
        throw std::invalid_argument("XYZ must be scalar FLOAT32 or FLOAT64 fields");
      xyz[i] = *it;
      const std::size_t size = it->datatype == Field::FLOAT32 ? 4 : 8;
      if (static_cast<std::size_t>(it->offset) + size > cloud.point_step)
        throw std::invalid_argument("XYZ field exceeds point_step");
    }
    if (cloud.point_step == 0 || static_cast<uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
        static_cast<uint64_t>(cloud.row_step) * cloud.height > cloud.data.size())
      throw std::invalid_argument("Invalid PointCloud2 layout/data length");
    for (const auto& field : cloud.fields) {
      if (field.name == "ring") {
        if (field.datatype != Field::UINT16 || field.count != 1 ||
            static_cast<uint64_t>(field.offset) + 2 > cloud.point_step)
          throw std::invalid_argument("ring must be a scalar UINT16 field");
        ring = &field;
      }
    }
  }

  template<typename T> T read(const uint8_t* pointer) const
  {
    std::array<uint8_t, sizeof(T)> bytes;
    std::copy_n(pointer, sizeof(T), bytes.data());
    if (swapBytes()) std::reverse(bytes.begin(), bytes.end());
    T value;
    std::memcpy(&value, bytes.data(), sizeof(T));
    return value;
  }

  template<typename T> void write(uint8_t* pointer, T value) const
  {
    std::array<uint8_t, sizeof(T)> bytes;
    std::memcpy(bytes.data(), &value, sizeof(T));
    if (swapBytes()) std::reverse(bytes.begin(), bytes.end());
    std::copy(bytes.begin(), bytes.end(), pointer);
  }

  Eigen::Vector3d point(std::size_t offset) const
  {
    Eigen::Vector3d p;
    for (std::size_t i = 0; i < 3; ++i) {
      const auto* bytes = cloud.data.data() + offset + xyz[i].offset;
      p[i] = xyz[i].datatype == Field::FLOAT32 ? read<float>(bytes) : read<double>(bytes);
    }
    return p;
  }

  void append(Cloud& out, std::size_t offset, const Eigen::Vector3d* transformed = nullptr) const
  {
    const auto begin = out.data.size();
    if (begin + cloud.point_step > std::numeric_limits<uint32_t>::max())
      throw std::invalid_argument("Output cloud exceeds PointCloud2 row_step range");
    out.data.insert(out.data.end(), cloud.data.begin() + offset,
                    cloud.data.begin() + offset + cloud.point_step);
    if (transformed) {
      for (std::size_t i = 0; i < 3; ++i) {
        auto* bytes = out.data.data() + begin + xyz[i].offset;
        if (xyz[i].datatype == Field::FLOAT32) write<float>(bytes, (*transformed)[i]);
        else write<double>(bytes, (*transformed)[i]);
      }
    }
    ++out.width;
    out.row_step = static_cast<uint32_t>(out.data.size());
  }

  Cloud empty(const std::string& frame) const
  {
    Cloud out;
    out.header = cloud.header;
    out.header.frame_id = frame;
    out.height = 1;
    out.fields = cloud.fields;
    out.is_bigendian = cloud.is_bigendian;
    out.point_step = cloud.point_step;
    out.is_dense = true;
    return out;
  }

  bool swapBytes() const
  {
    const uint16_t value = 1;
    const bool host_bigendian = *reinterpret_cast<const uint8_t*>(&value) == 0;
    return cloud.is_bigendian != host_bigendian;
  }

  const Cloud& cloud;
  std::array<Field, 3> xyz;
  const Field* ring{nullptr};
};
}  // namespace ground_seg
