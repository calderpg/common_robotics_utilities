#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <common_robotics_utilities/cru_namespace.hpp>

namespace common_robotics_utilities
{
CRU_NAMESPACE_BEGIN
namespace zlib_helpers
{
std::vector<uint8_t> DecompressBytes(const std::vector<uint8_t>& compressed);

std::vector<uint8_t> CompressBytes(const std::vector<uint8_t>& uncompressed);

std::vector<uint8_t> LoadFromFileAndDecompress(const std::string& filename);

void CompressAndWriteToFile(
    const std::vector<uint8_t>& uncompressed, const std::string& filename);
}  // namespace zlib_helpers
CRU_NAMESPACE_END
}  // namespace common_robotics_utilities
