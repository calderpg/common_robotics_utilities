#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <random>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace random_rotation_generator
{
/// Generator for uniform random quaternions and Euler angles.
class RandomRotationGenerator
{
private:
  std::uniform_real_distribution<double> uniform_unit_dist_;

public:
  RandomRotationGenerator() : uniform_unit_dist_(0.0, 1.0) {}

  // From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
  // see pages 124-132.
  static Eigen::Quaterniond GenerateUniformRandomQuaternion(
      const std::function<double()>& uniform_unit_dist)
  {
    const double x0 = uniform_unit_dist();
    const double r1 = std::sqrt(1.0 - x0);
    const double r2 = std::sqrt(x0);
    const double t1 = 2.0 * M_PI * uniform_unit_dist();
    const double t2 = 2.0 * M_PI * uniform_unit_dist();
    const double c1 = std::cos(t1);
    const double s1 = std::sin(t1);
    const double c2 = std::cos(t2);
    const double s2 = std::sin(t2);
    const double x = s1 * r1;
    const double y = c1 * r1;
    const double z = s2 * r2;
    const double w = c2 * r2;
    return Eigen::Quaterniond(w, x, y, z);
  }

  // From Effective Sampling and Distance Metrics for 3D Rigid Body Path
  // Planning, by James Kuffner, ICRA 2004.
  static Eigen::Vector3d GenerateUniformRandomEulerAngles(
      const std::function<double()>& uniform_unit_dist)
  {
    const double roll = 2.0 * M_PI * uniform_unit_dist() -  M_PI;
    const double pitch_init
        = std::acos(1.0 - (2.0 * uniform_unit_dist())) + M_PI_2;
    const double pitch
        = (uniform_unit_dist() < 0.5)
          ? ((pitch_init < M_PI) ? pitch_init + M_PI : pitch_init - M_PI)
          : pitch_init;
    const double yaw = 2.0 * M_PI * uniform_unit_dist() -  M_PI;
    return Eigen::Vector3d(roll, pitch, yaw);
  }

  template<typename Generator>
  Eigen::Quaterniond GetQuaternion(Generator& prng)
  {
    std::function<double()> uniform_rand_fn
        = [&] () { return uniform_unit_dist_(prng); };
    return GenerateUniformRandomQuaternion(uniform_rand_fn);
  }

  template<typename Generator>
  std::vector<double> GetRawQuaternion(Generator& prng)
  {
    const Eigen::Quaterniond quat = GetQuaternion(prng);
    return std::vector<double>{quat.x(), quat.y(), quat.z(), quat.w()};
  }

  template<typename Generator>
  Eigen::Vector3d GetEulerAngles(Generator& prng)
  {
    std::function<double()> uniform_rand_fn
        = [&] () { return uniform_unit_dist_(prng); };
    return GenerateUniformRandomEulerAngles(uniform_rand_fn);
  }

  template<typename Generator>
  std::vector<double> GetRawEulerAngles(Generator& prng)
  {
    const Eigen::Vector3d angles = GetEulerAngles(prng);
    return std::vector<double>{angles.x(), angles.y(), angles.z()};
  }
};
}  // namespace random_rotation_generator
}  // namespace common_robotics_utilities
