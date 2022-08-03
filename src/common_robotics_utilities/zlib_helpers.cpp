#include <common_robotics_utilities/zlib_helpers.hpp>

#include <zlib.h>

#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace common_robotics_utilities
{
namespace zlib_helpers
{
std::vector<uint8_t> DecompressBytes(const std::vector<uint8_t>& compressed)
{
  if (compressed.size() <= std::numeric_limits<uint32_t>::max())
  {
    z_stream strm;
    std::vector<uint8_t> buffer;
    const size_t BUFSIZE = 1024 * 1024;
    uint8_t temp_buffer[BUFSIZE];
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    int ret = inflateInit(&strm);
    if (ret != Z_OK)
    {
      inflateEnd(&strm);
      throw std::runtime_error(
          "ZLIB can't init inflate stream (ret = " + std::to_string(ret) + ")");
    }
    strm.avail_in = static_cast<uint32_t>(compressed.size());
    strm.next_in = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(
                       compressed.data()));
    do
    {
      strm.next_out = temp_buffer;
      strm.avail_out = BUFSIZE;
      ret = inflate(&strm, Z_NO_FLUSH);
      if (buffer.size() < strm.total_out)
      {
        buffer.insert(buffer.end(), temp_buffer,
                      temp_buffer + BUFSIZE - strm.avail_out);
      }
    }
    while (ret == Z_OK);
    if (ret != Z_STREAM_END)
    {
      inflateEnd(&strm);
      throw std::runtime_error(
          "ZLIB can't inflate stream (ret = " + std::to_string(ret) + ")");
    }
    inflateEnd(&strm);
    std::vector<uint8_t> decompressed(buffer);
    return decompressed;
  }
  else
  {
    throw std::invalid_argument("compressed too large");
  }
}

std::vector<uint8_t> CompressBytes(const std::vector<uint8_t>& uncompressed)
{
  if (uncompressed.size() <= std::numeric_limits<uint32_t>::max())
  {
    z_stream strm;
    std::vector<uint8_t> buffer;
    const size_t BUFSIZE = 1024 * 1024;
    uint8_t temp_buffer[BUFSIZE];
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = static_cast<uint32_t>(uncompressed.size());
    strm.next_in = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(
                       uncompressed.data()));
    strm.next_out = temp_buffer;
    strm.avail_out = BUFSIZE;
    int ret = deflateInit(&strm, Z_BEST_SPEED);
    if (ret != Z_OK)
    {
      deflateEnd(&strm);
      throw std::runtime_error(
          "ZLIB can't init deflate stream (ret = " + std::to_string(ret) + ")");
    }
    while (strm.avail_in != 0)
    {
      ret = deflate(&strm, Z_NO_FLUSH);
      if (ret != Z_OK)
      {
        deflateEnd(&strm);
        throw std::runtime_error(
            "ZLIB can't deflate stream (ret = " + std::to_string(ret) + ")");
      }
      if (strm.avail_out == 0)
      {
        buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
      }
    }
    int deflate_ret = Z_OK;
    while (deflate_ret == Z_OK)
    {
      if (strm.avail_out == 0)
      {
        buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
      }
      deflate_ret = deflate(&strm, Z_FINISH);
    }
    if (deflate_ret != Z_STREAM_END)
    {
      deflateEnd(&strm);
      throw std::runtime_error(
          "ZLIB can't deflate stream (ret = " + std::to_string(deflate_ret) +
          ")");
    }
    buffer.insert(buffer.end(), temp_buffer,
                  temp_buffer + BUFSIZE - strm.avail_out);
    deflateEnd(&strm);
    std::vector<uint8_t> compressed(buffer);
    return compressed;
  }
  else
  {
    throw std::invalid_argument("uncompressed too large");
  }
}

std::vector<uint8_t> LoadFromFileAndDecompress(const std::string& filename)
{
  std::ifstream input_file(
      filename, std::ios::binary | std::ios::in | std::ios::ate);
  if (!input_file.is_open())
  {
    throw std::runtime_error("Failed to open file [" + filename + "]");
  }
  std::streamsize size = input_file.tellg();
  input_file.seekg(0, std::ios::beg);
  std::vector<uint8_t> file_buffer(static_cast<size_t>(size));
  if (!(input_file.read(reinterpret_cast<char*>(file_buffer.data()), size)))
  {
    throw std::runtime_error("Failed to read entire contents of file");
  }
  const std::vector<uint8_t> decompressed = DecompressBytes(file_buffer);
  return decompressed;
}

void CompressAndWriteToFile(
    const std::vector<uint8_t>& uncompressed, const std::string& filename)
{
  const auto compressed = CompressBytes(uncompressed);
  std::ofstream output_file(filename, std::ios::out | std::ios::binary);
  output_file.write(reinterpret_cast<const char*>(compressed.data()),
                    static_cast<std::streamsize>(compressed.size()));
  output_file.close();
}
} // namespace zlib_helpers
} // namespace common_robotics_utilities
