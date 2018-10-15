#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace common_robotics_utilities
{
namespace base64_helpers
{
std::vector<uint8_t> Decode(const std::string& encoded);

std::string Encode(const std::vector<uint8_t>& binary);
}  // namespace base64_helpers
}  // namespace common_robotics_utilities
