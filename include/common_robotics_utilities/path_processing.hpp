#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <random>
#include <stdexcept>
#include <vector>

#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/utility.hpp>

namespace common_robotics_utilities
{
namespace path_processing
{
template<typename Configuration, typename Container=std::vector<Configuration>>
inline double ComputePercentCollisionFree(
  const Container& path,
  const std::function<bool(const Configuration&,
                           const Configuration&)>& edge_validity_check_fn)
{
  if (path.size() >= 2)
  {
    const size_t num_edges = path.size() - 1;
    size_t collision_free_edges = 0;
    for (size_t idx = 1; idx < path.size(); idx++)
    {
      const Configuration& q1 = path[idx - 1];
      const Configuration& q2 = path[idx];
      const bool edge_valid = edge_validity_check_fn(q1, q2);
      if (edge_valid)
      {
        collision_free_edges += 1;
      }
      else
      {
        break;
      }
    }
    return static_cast<double>(collision_free_edges)
        / static_cast<double>(num_edges);
  }
  else if (path.size() == 1)
  {
    if (edge_validity_check_fn(path.front(), path.front()))
    {
      return 1.0;
    }
    else
    {
      return 0.0;
    }
  }
  else
  {
    return 1.0;
  }
}

template<typename Configuration, typename Container=std::vector<Configuration>>
inline Container AttemptShortcut(
    const Container& current_path,
    const size_t start_index,
    const size_t end_index,
    const uint32_t remaining_backtracking_steps,
    const double resample_shortcuts_interval,
    const bool check_for_marginal_shortcuts,
    const std::function<bool(const Configuration&,
                             const Configuration&)>& edge_validity_check_fn,
    const std::function<double(const Configuration&,
                               const Configuration&)>& state_distance_fn,
    const std::function<Configuration(const Configuration&,
                                      const Configuration&,
                                      const double)>& state_interpolation_fn)
{
  // Check if the edge is valid
  if (start_index >= end_index)
  {
    throw std::invalid_argument("start_index >= end_index");
  }
  const Configuration& start_config = current_path[start_index];
  const Configuration& end_config = current_path[end_index];
  const bool edge_valid = edge_validity_check_fn(start_config, end_config);
  if (edge_valid)
  {
    // Make the shortcut
    Container shortcut;
    // Copy the start config
    shortcut.emplace_back(start_config);
    // Insert resampled states in the shortcut if needed
    if (resample_shortcuts_interval > 0.0)
    {
      const double distance = state_distance_fn(start_config, end_config);
      const double raw_num_intervals = distance / resample_shortcuts_interval;
      const uint32_t num_segments =
          static_cast<uint32_t>(std::ceil(raw_num_intervals));
      for (uint32_t segment = 1u; segment < num_segments; segment++)
      {
        const double interpolation_ratio
            = static_cast<double>(segment) / static_cast<double>(num_segments);
        shortcut.emplace_back(
            state_interpolation_fn(start_config, end_config,
                                   interpolation_ratio));
      }
    }
    // Copy end config
    shortcut.emplace_back(end_config);
    if (shortcut.size() <= 2 || !check_for_marginal_shortcuts)
    {
      return shortcut;
    }
    // Check if this was a marginal path that could clip obstacles
    else if (ComputePercentCollisionFree<Configuration, Container>(
                 shortcut, edge_validity_check_fn)
             == 1.0)
    {
      return shortcut;
    }
  }
  // If we haven't already returned, the single shortcut has failed
  if (remaining_backtracking_steps > 0)
  {
    const size_t window = end_index - start_index;
    if (window >= 2)
    {
      const size_t half_window = window / 2;
      const size_t middle_index = start_index + half_window;
      // Attempt to shortcut each half independently
      const uint32_t available_backtracking_steps
          = remaining_backtracking_steps - 1;
      const auto first_half_shortcut
          = AttemptShortcut<Configuration, Container>(
              current_path, start_index, middle_index,
              available_backtracking_steps, resample_shortcuts_interval,
              check_for_marginal_shortcuts, edge_validity_check_fn,
              state_distance_fn, state_interpolation_fn);
      const auto second_half_shortcut
          = AttemptShortcut<Configuration, Container>(
              current_path, middle_index, end_index,
              available_backtracking_steps, resample_shortcuts_interval,
              check_for_marginal_shortcuts, edge_validity_check_fn,
              state_distance_fn, state_interpolation_fn);
      Container shortcut;
      if (first_half_shortcut.size() > 0 && second_half_shortcut.size() > 0)
      {
        shortcut.insert(
            shortcut.end(), first_half_shortcut.begin(),
            first_half_shortcut.end());
        // Skip the first configuration, since this is a duplicate of the last
        // configuration in the first half shortcut
        shortcut.insert(
            shortcut.end(), second_half_shortcut.begin() + 1,
            second_half_shortcut.end());
      }
      else if (first_half_shortcut.size() > 0)
      {
        shortcut.insert(
            shortcut.end(), first_half_shortcut.begin(),
            first_half_shortcut.end());
        // Skip the middle configuration, since this is a duplicate of the
        // last configuration in the first half shortcut, but include the end
        // index
        shortcut.insert(
            shortcut.end(),
            current_path.begin() + static_cast<ptrdiff_t>(middle_index) + 1,
            current_path.begin() + static_cast<ptrdiff_t>(end_index) + 1);
      }
      else if (second_half_shortcut.size() > 0)
      {
        // Skip the middle configuration, since this is a duplicate of the
        // first configuration in the second half shortcut
        shortcut.insert(
            shortcut.end(),
            current_path.begin() + static_cast<ptrdiff_t>(start_index),
            current_path.begin() + static_cast<ptrdiff_t>(middle_index));
        shortcut.insert(
            shortcut.end(), second_half_shortcut.begin(),
            second_half_shortcut.end());
      }
      return shortcut;
    }
  }
  // If we get here, the shortcut failed and we return an empty shortcut path
  return Container();
}

template<typename Configuration,
         typename Container=std::vector<Configuration>>
inline Container ShortcutSmoothPath(
    const Container& path,
    const uint32_t max_iterations,
    const uint32_t max_failed_iterations,
    const uint32_t max_backtracking_steps,
    const double max_shortcut_fraction,
    const double resample_shortcuts_interval,
    const bool check_for_marginal_shortcuts,
    const std::function<bool(const Configuration&,
                             const Configuration&)>& edge_validity_check_fn,
    const std::function<double(const Configuration&,
                               const Configuration&)>& state_distance_fn,
    const std::function<Configuration(const Configuration&,
                                      const Configuration&,
                                      const double)>& state_interpolation_fn,
    const utility::UniformUnitRealFunction& uniform_unit_real_fn)
{
  Container current_path = path;
  uint32_t num_iterations = 0;
  uint32_t failed_iterations = 0;
  while (num_iterations < max_iterations
         && failed_iterations < max_failed_iterations
         && current_path.size() > 2)
  {
    num_iterations++;
    // Attempt a shortcut
    const int64_t base_index = utility::GetUniformRandomIndex(
        uniform_unit_real_fn, static_cast<int64_t>(current_path.size()));
    // Pick an offset fraction
    const double offset_fraction = math::Interpolate(
        -max_shortcut_fraction, max_shortcut_fraction, uniform_unit_real_fn());
    // Compute the offset index
    const int64_t offset_index
        = base_index + static_cast<int64_t>(std::floor(
            static_cast<double>(current_path.size()) * offset_fraction));
    // We need to clamp it to the bounds of the current path
    const int64_t safe_offset_index = utility::ClampValue(
        offset_index, INT64_C(0),
        static_cast<int64_t>(current_path.size() - 1));
    // Get start & end indices
    const size_t start_index =
        static_cast<size_t>(std::min(base_index, safe_offset_index));
    const size_t end_index =
        static_cast<size_t>(std::max(base_index, safe_offset_index));
    if (end_index <= start_index + 1)
    {
      continue;
    }
    const auto shortcut = AttemptShortcut<Configuration, Container>(
        current_path, start_index, end_index, max_backtracking_steps,
        resample_shortcuts_interval, check_for_marginal_shortcuts,
        edge_validity_check_fn, state_distance_fn, state_interpolation_fn);
    // An empty shortcut means it failed, since the shortcut must include the
    // start and end configurations
    if (shortcut.size() > 0)
    {
      Container shortened_path;
      if (start_index > 0)
      {
        // Copy the path before the shortcut (excluding start_index)
        shortened_path.insert(
            shortened_path.end(), current_path.begin(),
            current_path.begin() + static_cast<ptrdiff_t>(start_index));
      }
      // Copy the shortcut
      shortened_path.insert(
          shortened_path.end(), shortcut.begin(), shortcut.end());
      if (end_index < current_path.size() - 1)
      {
        // Copy the path after the shortcut (excluding end_index)
        shortened_path.insert(
            shortened_path.end(),
            current_path.begin() + static_cast<ptrdiff_t>(end_index) + 1,
            current_path.end());
      }
      // Swap in as the new current path
      current_path = shortened_path;
    }
    else
    {
      failed_iterations++;
    }
  }
  return current_path;
}

template<typename Configuration, typename Container=std::vector<Configuration>>
inline Container ResamplePath(
    const Container& path,
    const double resampled_state_distance,
    const std::function<double(const Configuration&,
                               const Configuration&)>& state_distance_fn,
    const std::function<Configuration(const Configuration&,
                                      const Configuration&,
                                      const double)>& state_interpolation_fn)
{
  if (path.size() <= 1)
  {
    return path;
  }
  if (resampled_state_distance <= 0.0)
  {
    throw std::invalid_argument("resampled_state_distance must be > 0");
  }
  Container resampled_path;
  // Add the first state
  resampled_path.push_back(path[0]);
  // Loop through the path, adding interpolated states as needed
  for (size_t idx = 1; idx < path.size(); idx++)
  {
    // Get the states from the original path
    const Configuration& previous_state = path[idx - 1];
    const Configuration& current_state = path[idx];
    // We want to add all the intermediate states to our returned path
    const double distance = state_distance_fn(previous_state, current_state);
    const double raw_num_intervals = distance / resampled_state_distance;
    const uint32_t num_segments
        = static_cast<uint32_t>(std::ceil(raw_num_intervals));
    // If there's only one segment, we just add the end state of the window
    if (num_segments == 0u)
    {
      // Do nothing because this means distance was exactly 0
    }
    else if (num_segments == 1u)
    {
      // Add a single point for the other end of the segment
      resampled_path.push_back(current_state);
    }
    // If there is more than one segment, interpolate between previous_state and
    // current_state (including the current_state)
    else
    {
      for (uint32_t segment = 1u; segment <= num_segments; segment++)
      {
        const double interpolation_ratio
            = static_cast<double>(segment) / static_cast<double>(num_segments);
        const Configuration interpolated_state
            = state_interpolation_fn(
                previous_state, current_state, interpolation_ratio);
        resampled_path.push_back(interpolated_state);
      }
    }
  }
  return resampled_path;
}
}  // namespace path_processing
}  // namespace common_robotics_utilities
