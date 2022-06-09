#include <common_robotics_utilities/base64_helpers.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace common_robotics_utilities
{
namespace base64_helpers
{
/// Implementations derived from post at http://stackoverflow.com/a/41094722
static const std::array<int32_t, 256> B64IndexTable =
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 62, 63, 62, 62, 63,
     52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4,
     5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
     25, 0, 0, 0, 0, 63, 0, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
     39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};

static const std::array<char, 64> B64ValueTable =
    {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
     'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd',
     'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's',
     't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7',
     '8', '9', '+', '/'};

std::vector<uint8_t> Decode(const std::string& encoded)
{
  const size_t encoded_length = encoded.size();
  const size_t pad = encoded_length > 0 &&
                     (encoded_length % 4 || encoded[encoded_length - 1] == '=');
  const size_t L = ((encoded_length + 3) / 4 - pad) * 4;
  std::vector<uint8_t> buffer(L / 4 * 3 + pad, 0x00);
  for (size_t i = 0, j = 0; i < L; i += 4)
  {
    const int32_t n =
        B64IndexTable[static_cast<size_t>(encoded[i])] << 18
        | B64IndexTable[static_cast<size_t>(encoded[i + 1])] << 12
        | B64IndexTable[static_cast<size_t>(encoded[i + 2])] << 6
        | B64IndexTable[static_cast<size_t>(encoded[i + 3])];
    buffer[j++] = static_cast<uint8_t>(n >> 16);
    buffer[j++] = static_cast<uint8_t>(n >> 8 & 0xFF);
    buffer[j++] = static_cast<uint8_t>(n & 0xFF);
  }
  if (pad > 0)
  {
    int32_t n =
        B64IndexTable[static_cast<size_t>(encoded[L])] << 18
        | B64IndexTable[static_cast<size_t>(encoded[L + 1])] << 12;
    buffer[buffer.size() - 1] = static_cast<uint8_t>(n >> 16);
    if (encoded_length > L + 2 && encoded[L + 2] != '=')
    {
      n |= B64IndexTable[static_cast<size_t>(encoded[L + 2])] << 6;
      buffer.push_back(static_cast<uint8_t>(n >> 8 & 0xFF));
    }
  }
  return buffer;
}

std::string Encode(const std::vector<uint8_t>& binary)
{
  const size_t binary_length = binary.size();
  /* 3-byte blocks to 4-byte */
  const size_t encoded_length = 4 * ((binary_length + 2) / 3);
  if (encoded_length < binary_length)
  {
    return std::string(); /* integer overflow */
  }
  std::string encoded;
  encoded.resize(encoded_length);
  size_t input_position = 0;
  size_t output_position = 0;
  while (binary_length - input_position >= 3)
  {
    encoded[output_position++] = B64ValueTable[
        static_cast<size_t>(binary[input_position + 0] >> 2)];
    encoded[output_position++] = B64ValueTable[
        static_cast<size_t>(
            ((binary[input_position + 0] & 0x03) << 4)
            | (binary[input_position + 1] >> 4))];
    encoded[output_position++] = B64ValueTable[
        static_cast<size_t>(
            ((binary[input_position + 1] & 0x0f) << 2)
            | (binary[input_position + 2] >> 6))];
    encoded[output_position++] = B64ValueTable[
        static_cast<size_t>(binary[input_position + 2] & 0x3f)];
    input_position += 3;
  }
  if (input_position < binary_length)
  {
    encoded[output_position++] = B64ValueTable[
        static_cast<size_t>(binary[input_position + 0] >> 2)];
    if ((binary_length - input_position) == 1)
    {
      encoded[output_position++] = B64ValueTable[
          static_cast<size_t>((binary[input_position + 0] & 0x03) << 4)];
      encoded[output_position++] = '=';
    }
    else
    {
      encoded[output_position++] = B64ValueTable[
          static_cast<size_t>(
              ((binary[input_position + 0] & 0x03) << 4)
              | (binary[input_position + 1] >> 4))];
      encoded[output_position++] = B64ValueTable[
          static_cast<size_t>((binary[input_position + 1] & 0x0f) << 2)];
    }
    encoded[output_position++] = '=';
  }
  return encoded;
}
}  // namespace base64_helpers
}  // namespace common_robotics_utilities
