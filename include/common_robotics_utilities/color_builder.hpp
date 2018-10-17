#pragma once

#include <cmath>
#include <cstdint>

#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace color_builder
{
/// Convert @param hexval color channel to float.
inline constexpr float ColorChannelFromHex(uint8_t hexval)
{
  return static_cast<float>(hexval) / 255.0f;
}

/// Clamp @param val color channel to [0, 1].
inline float TrimColorValue(const float val)
{
  return utility::ClampValue<float>(val, 0.0f, 1.0f);
}

/// Convert @param colorval to hex value.
inline uint8_t ColorChannelToHex(float colorval)
{
  return static_cast<uint8_t>(std::round(TrimColorValue(colorval) * 255.0f));
}

/// Simple RGBA color storage type, meant to be API-compatible with other color
/// types like ROS's std_msgs::ColorRGBA.
class RGBAColor
{
public:

  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  float a = 0.0f;

  RGBAColor(const float in_r, const float in_g, const float in_b,
            const float in_a)
    : r(TrimColorValue(in_r)), g(TrimColorValue(in_g)),
      b(TrimColorValue(in_b)), a(TrimColorValue(in_a)) {}

  RGBAColor(const float in_r, const float in_g, const float in_b)
    : r(TrimColorValue(in_r)), g(TrimColorValue(in_g)),
      b(TrimColorValue(in_b)), a(1.0f) {}

  RGBAColor(const uint8_t in_r, const uint8_t in_g, const uint8_t in_b,
            const uint8_t in_a)
    : r(ColorChannelFromHex(in_r)), g(ColorChannelFromHex(in_g)),
      b(ColorChannelFromHex(in_b)), a(ColorChannelFromHex(in_a)) {}

  RGBAColor(const uint8_t in_r, const uint8_t in_g, const uint8_t in_b,
            const float in_a)
    : r(ColorChannelFromHex(in_r)), g(ColorChannelFromHex(in_g)),
      b(ColorChannelFromHex(in_b)), a(TrimColorValue(in_a)) {}

  RGBAColor(const uint8_t in_r, const uint8_t in_g, const uint8_t in_b)
    : r(ColorChannelFromHex(in_r)), g(ColorChannelFromHex(in_g)),
      b(ColorChannelFromHex(in_b)), a(1.0f) {}

  RGBAColor() : r(0.0f), g(0.0f), b(0.0f), a(0.0f) {}

  float R() const { return r; }

  float G() const { return g; }

  float B() const { return b; }

  float A() const { return a; }

  void SetR(const float new_r) { r = TrimColorValue(new_r); }

  void SetG(const float new_g) { g = TrimColorValue(new_g); }

  void SetB(const float new_b) { b = TrimColorValue(new_b); }

  void SetA(const float new_a) { a = TrimColorValue(new_a); }

  uint8_t GetRHex() const { return ColorChannelToHex(r); }

  uint8_t GetGHex() const { return ColorChannelToHex(g); }

  uint8_t GetBHex() const { return ColorChannelToHex(b); }

  uint8_t GetAHex() const { return ColorChannelToHex(a); }

  void SetRHex(const uint8_t hex_r) { r = ColorChannelFromHex(hex_r); }

  void SetGHex(const uint8_t hex_g) { g = ColorChannelFromHex(hex_g); }

  void SetBHex(const uint8_t hex_b) { b = ColorChannelFromHex(hex_b); }

  void SetAHex(const uint8_t hex_a) { a = ColorChannelFromHex(hex_a); }
};

/// Make a color value of ColorType from the provided float color chanel values.
template<typename ColorType>
inline ColorType MakeFromFloatColors(
    const float r, const float g, const float b, const float a=1.0f)
{
  ColorType color;
  color.r = TrimColorValue(r);
  color.g = TrimColorValue(g);
  color.b = TrimColorValue(b);
  color.a = TrimColorValue(a);
  return color;
}

/// Make a color value of ColorType from the provided hex color chanel values.
template<typename ColorType>
inline ColorType MakeFromHexColors(
    const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a=0xff)
{
  return MakeFromFloatColors<ColorType>(
      ColorChannelFromHex(r), ColorChannelFromHex(g), ColorChannelFromHex(b),
      ColorChannelFromHex(a));
}

/// Make a color value of ColorType from the provided color chanel values. This
/// covers the common case of colors specified in RGB hex but with a variable
/// alpha computed separately.
template<typename ColorType>
inline ColorType MakeFromMixedColors(
    const uint8_t r, const uint8_t g, const uint8_t b, const float a=1.0f)
{
  return MakeFromFloatColors<ColorType>(
      ColorChannelFromHex(r), ColorChannelFromHex(g), ColorChannelFromHex(b),
      TrimColorValue(a));
}

/// Interpolates a color in the "hot-to-cold" pattern for @param value given
/// lower value @param min_value and upper value @param max_value.
template<typename ColorType>
inline ColorType InterpolateHotToCold(
    const double value, const double min_value=0.0, const double max_value=1.0)
{
  const double real_value = utility::ClampValue(value, min_value, max_value);
  const double range = max_value - min_value;
  // Start with white
  double r = 1.0;
  double g = 1.0;
  double b = 1.0;
  // Interpolate
  if (real_value < (min_value + (0.25 * range)))
  {
    r = 0.0;
    g = 4.0 * (real_value - min_value) / range;
  }
  else if (real_value < (min_value + (0.5 * range)))
  {
    r = 0.0;
    b = 1.0 + 4.0 * (min_value + 0.25 * range - real_value) / range;
  }
  else if (real_value < (min_value + (0.75 * range)))
  {
    r = 4.0 * (real_value - min_value - 0.5 * range) / range;
    b = 0.0;
  }
  else
  {
    g = 1.0 + 4.0 * (min_value + 0.75 * range - real_value) / range;
    b = 0.0;
  }
  return MakeFromFloatColors<ColorType>(
      static_cast<float>(r), static_cast<float>(g), static_cast<float>(b),
      1.0f);
}

/// Converts between two color types wth float (or float-like) R,G,B,A members.
template<typename ColorTypeA, typename ColorTypeB>
inline ColorTypeB ConvertColor(const ColorTypeA& color)
{
  ColorTypeB cvt_color;
  cvt_color.r = TrimColorValue(color.r);
  cvt_color.g = TrimColorValue(color.g);
  cvt_color.b = TrimColorValue(color.b);
  cvt_color.a = TrimColorValue(color.a);
  return cvt_color;
}

/// Checks if two color types wth float (or float-like) R,G,B,A members are
/// equal to the provided tolerance.
template<typename ColorTypeA, typename ColorTypeB>
inline bool ColorsEqual(const ColorTypeA& color1, const ColorTypeB& color2,
                        const float tolerance=0.0f)
{
  if (std::abs(color1.r - color2.r) > tolerance)
  {
    return false;
  }
  if (std::abs(color1.g - color2.g) > tolerance)
  {
    return false;
  }
  if (std::abs(color1.b - color2.b) > tolerance)
  {
    return false;
  }
  if (std::abs(color1.a - color2.a) > tolerance)
  {
    return false;
  }
  return true;
}

/// Finds the "unique color" for the provided color code @param color_code, for
/// all color codes <= 20, the color is derived from a list of
/// percepturally-distinct colors. Beyond 20, black is returned. @param alpha
/// specifies the alpha channel of the color.
template<typename ColorType>
inline ColorType LookupUniqueColor(const uint32_t color_code,
                                   const float alpha=1.0f)
{
  if (color_code == 0)
  {
    return MakeFromFloatColors<ColorType>(1.0f, 1.0f, 1.0f, 0.0f);
  }
  if (color_code == 1)
  {
    return MakeFromMixedColors<ColorType>(0xff, 0x00, 0xb3, alpha);
  }
  else if (color_code == 2)
  {
    return MakeFromMixedColors<ColorType>(0x80, 0x75, 0x3e, alpha);
  }
  else if (color_code == 3)
  {
    return MakeFromMixedColors<ColorType>(0xff, 0x00, 0x68, alpha);
  }
  else if (color_code == 4)
  {
    return MakeFromMixedColors<ColorType>(0xa6, 0xd7, 0xbd, alpha);
  }
  else if (color_code == 5)
  {
    return MakeFromMixedColors<ColorType>(0xc1, 0x20, 0x00, alpha);
  }
  else if (color_code == 6)
  {
    return MakeFromMixedColors<ColorType>(0xce, 0x62, 0xa2, alpha);
  }
  else if (color_code == 7)
  {
    return MakeFromMixedColors<ColorType>(0x81, 0x66, 0x70, alpha);
  }
  else if (color_code == 8)
  {
    return MakeFromMixedColors<ColorType>(0x00, 0x34, 0x7d, alpha);
  }
  else if (color_code == 9)
  {
    return MakeFromMixedColors<ColorType>(0xf6, 0x8e, 0x76, alpha);
  }
  else if (color_code == 10)
  {
    return MakeFromMixedColors<ColorType>(0x00, 0x8a, 0x53, alpha);
  }
  else if (color_code == 11)
  {
    return MakeFromMixedColors<ColorType>(0xff, 0x5c, 0x7a, alpha);
  }
  else if (color_code == 12)
  {
    return MakeFromMixedColors<ColorType>(0x53, 0x7a, 0x37, alpha);
  }
  else if (color_code == 13)
  {
    return MakeFromMixedColors<ColorType>(0xff, 0x00, 0x8e, alpha);
  }
  else if (color_code == 14)
  {
    return MakeFromMixedColors<ColorType>(0xb3, 0x51, 0x28, alpha);
  }
  else if (color_code == 15)
  {
    return MakeFromMixedColors<ColorType>(0xf4, 0x00, 0xc8, alpha);
  }
  else if (color_code == 16)
  {
    return MakeFromMixedColors<ColorType>(0x7f, 0x0d, 0x18, alpha);
  }
  else if (color_code == 17)
  {
    return MakeFromMixedColors<ColorType>(0x93, 0x00, 0xaa, alpha);
  }
  else if (color_code == 18)
  {
    return MakeFromMixedColors<ColorType>(0x59, 0x15, 0x33, alpha);
  }
  else if (color_code == 19)
  {
    return MakeFromMixedColors<ColorType>(0xf1, 0x13, 0x3a, alpha);
  }
  else if (color_code == 20)
  {
    return MakeFromMixedColors<ColorType>(0x23, 0x16, 0x2c, alpha);
  }
  else
  {
    return MakeFromFloatColors<ColorType>(0.0f, 0.0f, 0.0f, alpha);
  }
}
}  // namespace color_builder
}  // namespace common_robotics_utilities
