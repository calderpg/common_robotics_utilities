#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <common_robotics_utilities/cru_namespace.hpp>

namespace common_robotics_utilities
{
CRU_NAMESPACE_BEGIN
namespace base64_helpers
{
std::vector<uint8_t> Decode(const std::string& encoded);

std::string Encode(const std::vector<uint8_t>& binary);
}  // namespace base64_helpers
CRU_NAMESPACE_END
}  // namespace common_robotics_utilities
