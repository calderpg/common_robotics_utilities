#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/path_processing.hpp>
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/simple_astar_search.hpp>
#include <common_robotics_utilities/simple_graph.hpp>
#include <common_robotics_utilities/simple_graph_search.hpp>
#include <common_robotics_utilities/simple_knearest_neighbors.hpp>
#include <common_robotics_utilities/simple_prm_planner.hpp>
#include <common_robotics_utilities/simple_rrt_planner.hpp>
#include <common_robotics_utilities/zlib_helpers.hpp>

#include <gtest/gtest.h>

namespace common_robotics_utilities
{
namespace planning_test
{
using TestMap = Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic>;
using Waypoint = std::pair<ssize_t, ssize_t>;
using WaypointAllocator = std::allocator<Waypoint>;
using WaypointVector = std::vector<Waypoint, WaypointAllocator>;
using WaypointPlannerTree = simple_rrt_planner::SimpleRRTPlannerTree<Waypoint>;

bool WaypointsEqual(const Waypoint& first, const Waypoint& second)
{
  return (first.first == second.first && first.second == second.second);
}

struct WaypointEqualer
{
  bool operator()(const Waypoint& first, const Waypoint& second) const
  {
    return WaypointsEqual(first, second);
  }
};

struct WaypointHasher
{
  size_t operator()(const Waypoint& waypoint) const
  {
    std::size_t hash_val = 0;
    common_robotics_utilities::utility::hash_combine(
        hash_val, waypoint.first, waypoint.second);
    return hash_val;
  }
};

double WaypointDistance(const Waypoint& start, const Waypoint& end)
{
  const double delta_rows = static_cast<double>(end.first - start.first);
  const double delta_cols = static_cast<double>(end.second - start.second);
  return std::sqrt((delta_rows * delta_rows) + (delta_cols * delta_cols));
}

Waypoint InterpolateWaypoint(
    const Waypoint& start, const Waypoint& end, const double ratio)
{
  const double real_ratio = utility::ClampValue(ratio, 0.0, 1.0);
  const double delta_rows = static_cast<double>(end.first - start.first);
  const double delta_cols = static_cast<double>(end.second - start.second);
  const double raw_interp_rows = delta_rows * real_ratio;
  const double raw_interp_cols = delta_cols * real_ratio;
  const ssize_t interp_row
      = start.first + static_cast<ssize_t>(std::round(raw_interp_rows));
  const ssize_t interp_col
      = start.second + static_cast<ssize_t>(std::round(raw_interp_cols));
  return Waypoint(interp_row, interp_col);
}

WaypointVector ResampleWaypoints(const WaypointVector& waypoints)
{
  return path_processing::ResamplePath<Waypoint>(
      waypoints, 0.5, WaypointDistance, InterpolateWaypoint);
}

void DrawEnvironment(const TestMap& environment)
{
  std::cout << environment << std::endl;
}

void SetCell(TestMap& map, const ssize_t row, const ssize_t col, const char val)
{
  if (map(row, col) != '#')
  {
    map(row, col) = val;
  }
}

void DrawPaths(
    const TestMap& environment,
    const WaypointVector& starts,
    const WaypointVector& goals,
    const std::vector<WaypointVector>& paths)
{
  TestMap working_copy = environment;
  for (const auto& path : paths)
  {
    if (path.size() > 0)
    {
      SetCell(working_copy, path.at(0).first, path.at(0).second, '+');
      for (size_t idx = 1; idx < path.size(); idx++)
      {
        const Waypoint& previous = path.at(idx - 1);
        const Waypoint& current = path.at(idx);
        const auto edge_path = ResampleWaypoints({previous, current});
        for (const auto& state : edge_path)
        {
          const char current_val = working_copy(state.first, state.second);
          if (current_val != '+')
          {
            SetCell(working_copy, state.first, state.second, '-');
          }
        }
        SetCell(working_copy, current.first, current.second, '+');
      }
    }
  }
  for (const auto& start : starts)
  {
    SetCell(working_copy, start.first, start.second, 'S');
  }
  for (const auto& goal : goals)
  {
    SetCell(working_copy, goal.first, goal.second, 'G');
  }
  DrawEnvironment(working_copy);
}

void DrawPath(
    const TestMap& environment,
    const WaypointVector& starts,
    const WaypointVector& goals,
    const WaypointVector& path)
{
  return DrawPaths(environment, starts, goals, {path});
}

TestMap MakeTestMap(
    const std::string& test_map_string, const ssize_t rows, const ssize_t cols)
{
  if (static_cast<ssize_t>(test_map_string.size()) != (rows * cols))
  {
    throw std::invalid_argument("test_map_string is the wrong size");
  }
  TestMap test_map(rows, cols);
  memcpy(test_map.data(), test_map_string.data(),
         test_map_string.size() * sizeof(char));
  test_map.transposeInPlace();
  return test_map;
}

bool CheckWaypointCollisionFree(const TestMap& map, const Waypoint& waypoint)
{
  return (map(waypoint.first, waypoint.second) != '#');
}

bool CheckEdgeCollisionFree(
    const TestMap& map, const Waypoint& start, const Waypoint& end,
    const double step_size)
{
  const double distance = WaypointDistance(start, end);
  const double raw_num_intervals = distance / step_size;
  const int32_t num_states
      = std::max(static_cast<int32_t>(std::ceil(raw_num_intervals)), 1);
  for (int32_t state = 0; state <= num_states; state++)
  {
    const double interpolation_ratio
        = static_cast<double>(state) / static_cast<double>(num_states);
    const Waypoint interpolated
        = InterpolateWaypoint(start, end, interpolation_ratio);
    if (!CheckWaypointCollisionFree(map, interpolated))
    {
      return false;
    }
  }
  return true;
}

WaypointVector SmoothWaypoints(
    const WaypointVector& waypoints,
    const std::function<bool(const Waypoint&, const Waypoint&)>& check_edge_fn,
    const utility::UniformUnitRealFunction& uniform_unit_real_fn)
{
  // Parameters for shortcut smoothing
  const uint32_t max_iterations = 100;
  const uint32_t max_failed_iterations = 100;
  const uint32_t max_backtracking_steps = 1;
  const double max_shortcut_fraction = 0.5;
  const double resample_shortcuts_interval = 0.5;
  const bool check_for_marginal_shortcuts = false;
  return path_processing::ShortcutSmoothPath<Waypoint>(
      waypoints, max_iterations, max_failed_iterations, max_backtracking_steps,
      max_shortcut_fraction, resample_shortcuts_interval,
      check_for_marginal_shortcuts, check_edge_fn, WaypointDistance,
      InterpolateWaypoint, uniform_unit_real_fn);
}

void DrawRoadmap(
    const TestMap& environment,
    const simple_graph::Graph<Waypoint>& roadmap)
{
  TestMap working_copy = environment;
  const auto& roadmap_nodes = roadmap.GetNodesImmutable();
  for (const auto& roadmap_node : roadmap_nodes)
  {
    const auto& out_edges = roadmap_node.GetOutEdgesImmutable();
    for (const auto& edge : out_edges)
    {
      const Waypoint& self
          = roadmap.GetNodeImmutable(edge.GetFromIndex()).GetValueImmutable();
      const Waypoint& other
          = roadmap.GetNodeImmutable(edge.GetToIndex()).GetValueImmutable();
      const auto edge_path = ResampleWaypoints({self, other});
      for (const auto& waypoint : edge_path)
      {
        const char current_val = working_copy(waypoint.first, waypoint.second);
        if (current_val != '+')
        {
          SetCell(working_copy, waypoint.first, waypoint.second, '-');
        }
      }
      SetCell(working_copy, self.first, self.second, '+');
      SetCell(working_copy, other.first, other.second, '+');
    }
  }
  DrawEnvironment(working_copy);
}

WaypointVector GenerateAllPossible8ConnectedChildren(
    const Waypoint& waypoint)
{
  return WaypointVector{
      Waypoint(waypoint.first - 1, waypoint.second - 1),
      Waypoint(waypoint.first - 1, waypoint.second),
      Waypoint(waypoint.first - 1, waypoint.second + 1),
      Waypoint(waypoint.first, waypoint.second - 1),
      Waypoint(waypoint.first, waypoint.second + 1),
      Waypoint(waypoint.first + 1, waypoint.second - 1),
      Waypoint(waypoint.first + 1, waypoint.second),
      Waypoint(waypoint.first + 1, waypoint.second + 1)};
}

Waypoint SampleWaypoint(
    const TestMap& map,
    const utility::UniformUnitRealFunction& uniform_unit_real_fn)
{
  const ssize_t row = utility::GetUniformRandomInRange<ssize_t>(
      uniform_unit_real_fn, 1, map.rows() - 1);
  const ssize_t col = utility::GetUniformRandomInRange<ssize_t>(
      uniform_unit_real_fn, 1, map.cols() - 1);
  return Waypoint(row, col);
}

GTEST_TEST(PlanningTest, Test)
{
  const std::string test_env_raw = "####################"
                                   "#                  #"
                                   "#  ####            #"
                                   "#  ####    #####   #"
                                   "#  ####    #####   #"
                                   "#          #####   #"
                                   "#          #####   #"
                                   "#                  #"
                                   "#      #########   #"
                                   "#     ##########   #"
                                   "#    ###########   #"
                                   "#   ############   #"
                                   "#                  #"
                                   "#                  #"
                                   "#    ##            #"
                                   "#    ##   ######## #"
                                   "#    ##   ######## #"
                                   "#    ##   ######## #"
                                   "#                  #"
                                   "####################";
  const TestMap test_env = MakeTestMap(test_env_raw, 20, 20);
  std::cout << "Planning environment" << std::endl;
  DrawEnvironment(test_env);

  const int64_t prng_seed = 42;
  std::mt19937_64 prng(prng_seed);
  std::uniform_real_distribution<double> uniform_unit_dist(0.0, 1.0);
  utility::UniformUnitRealFunction uniform_unit_real_fn = [&] ()
  {
    return uniform_unit_dist(prng);
  };

  const WaypointVector keypoints
      = {Waypoint(1, 1), Waypoint(18, 18), Waypoint(7, 13), Waypoint(9, 5)};

  // Bind helper functions used by multiple planners
  const std::function<bool(const Waypoint&)> check_state_validity_fn
      = [&] (const Waypoint& waypoint)
  {
    return CheckWaypointCollisionFree(test_env, waypoint);
  };

  const std::function<bool(const Waypoint&, const Waypoint&)>
      check_edge_validity_fn = [&] (const Waypoint& start, const Waypoint& end)
  {
    // We check both forward and backward because rounding in the waypoint
    // interpolation can create edges that are valid in only one direction.
    return (CheckEdgeCollisionFree(test_env, start, end, 0.5) &&
            CheckEdgeCollisionFree(test_env, end, start, 0.5));
  };

  const std::function<Waypoint(void)> state_sampling_fn = [&] (void)
  {
    return SampleWaypoint(test_env, uniform_unit_real_fn);
  };

  // Functions to check planning results
  const std::function<void(const WaypointVector&)> check_path =
      [&] (const WaypointVector& path)
  {
    ASSERT_GE(static_cast<int32_t>(path.size()), 2);
    for (size_t idx = 1; idx < path.size(); idx++)
    {
      // We check both forward and backward because rounding in the waypoint
      // interpolation can create edges that are valid in only one direction.
      const bool forward_valid =
          check_edge_validity_fn(path.at(idx - 1), path.at(idx));
      const bool backward_valid =
          check_edge_validity_fn(path.at(idx), path.at(idx - 1));
      const bool edge_valid = forward_valid && backward_valid;
      ASSERT_TRUE(edge_valid);
    }
  };

  const std::function<void(
      const TestMap&, const WaypointVector&,
      const WaypointVector&, const WaypointVector&)> check_plan =
      [&] (const TestMap& environment, const WaypointVector& starts,
           const WaypointVector& goals,
           const WaypointVector& path)
  {
    DrawPath(environment, starts, goals, path);
    std::cout << "Checking raw path" << std::endl;
    check_path(path);
    const auto smoothed_path =
        SmoothWaypoints(path, check_edge_validity_fn, uniform_unit_real_fn);
    std::cout << "Checking smoothed path" << std::endl;
    check_path(smoothed_path);
    const auto resampled_path = ResampleWaypoints(smoothed_path);
    std::cout << "Checking resampled path" << std::endl;
    check_path(resampled_path);
  };

  // RRT and BiRRT parameters
  const double rrt_step_size = 3.0;
  const double rrt_goal_bias = 0.1;
  const double rrt_timeout = 5.0;
  const double birrt_tree_sampling_bias = 0.5;
  const double birrt_p_switch_trees = 0.25;

  // Make RRT helpers
  auto rrt_nearest_neighbors_fn
      = simple_rrt_planner
          ::MakeKinematicLinearRRTNearestNeighborsFunction<Waypoint>(
              WaypointDistance, false);
  auto rrt_extend_fn
      = simple_rrt_planner::MakeKinematicRRTExtendPropagationFunction<Waypoint>(
          WaypointDistance, InterpolateWaypoint, check_edge_validity_fn,
          rrt_step_size);
  auto rrt_connect_fn
      = simple_rrt_planner
          ::MakeKinematicRRTConnectPropagationFunction<Waypoint>(
              WaypointDistance, InterpolateWaypoint, check_edge_validity_fn,
              rrt_step_size);
  auto select_sample_type_fn
      = simple_rrt_planner
          ::MakeUniformRandomBiRRTSelectSampleTypeFunction<Waypoint>(
              uniform_unit_real_fn, birrt_tree_sampling_bias);
  auto tree_sampling_fn
      = simple_rrt_planner
          ::MakeUniformRandomBiRRTTreeSamplingFunction<Waypoint>(
              uniform_unit_real_fn);
  auto birrt_nearest_neighbors_fn
      = simple_rrt_planner
          ::MakeKinematicLinearBiRRTNearestNeighborsFunction<Waypoint>(
              WaypointDistance, false);
  auto birrt_extend_fn
      = simple_rrt_planner
          ::MakeKinematicBiRRTExtendPropagationFunction<Waypoint>(
              WaypointDistance, InterpolateWaypoint, check_edge_validity_fn,
              rrt_step_size);
  auto birrt_connect_fn
      = simple_rrt_planner
          ::MakeKinematicBiRRTConnectPropagationFunction<Waypoint>(
              WaypointDistance, InterpolateWaypoint, check_edge_validity_fn,
              rrt_step_size);
  const simple_rrt_planner::BiRRTStatesConnectedFunction<Waypoint>
      birrt_states_connected_fn = [] (
          const Waypoint& first, const Waypoint& second,
          const simple_rrt_planner::BiRRTActiveTreeType)
  {
    return WaypointsEqual(first, second);
  };
  auto select_active_tree_fn
      = simple_rrt_planner
          ::MakeUniformRandomBiRRTSelectActiveTreeFunction<Waypoint>(
              uniform_unit_real_fn, birrt_p_switch_trees);

  // Grow a roadmap for the environment
  const int64_t K = 5;
  const int64_t grown_roadmap_size = 100;
  const std::function<bool(const int64_t)> grown_roadmap_termination_fn
      = [] (const int64_t current_roadmap_size)
  {
    return (current_roadmap_size >= grown_roadmap_size);
  };
  simple_graph::Graph<Waypoint> grown_roadmap;
  simple_prm_planner::GrowRoadMap<Waypoint>(
      grown_roadmap, state_sampling_fn, WaypointDistance,
      check_state_validity_fn, check_edge_validity_fn,
      grown_roadmap_termination_fn, K, false, true, false);
  ASSERT_TRUE(grown_roadmap.CheckGraphLinkage());
  std::cout << "Roadmap grown" << std::endl;
  simple_prm_planner::UpdateRoadMapEdges<Waypoint>(
      grown_roadmap, check_edge_validity_fn, WaypointDistance, false);
  ASSERT_TRUE(grown_roadmap.CheckGraphLinkage());
  std::cout << "Roadmap (grown) updated" << std::endl;

  // Build a roadmap for the environment
  const int64_t built_roadmap_size = 100;
  simple_graph::Graph<Waypoint> built_roadmap =
      simple_prm_planner::BuildRoadMap<Waypoint, simple_graph::Graph<Waypoint>>(
      built_roadmap_size, state_sampling_fn, WaypointDistance,
      check_state_validity_fn, check_edge_validity_fn, K, 100, true, false,
      false);
  ASSERT_TRUE(built_roadmap.CheckGraphLinkage());
  std::cout << "Roadmap built" << std::endl;
  simple_prm_planner::UpdateRoadMapEdges<Waypoint>(
      built_roadmap, check_edge_validity_fn, WaypointDistance, false);
  ASSERT_TRUE(built_roadmap.CheckGraphLinkage());
  std::cout << "Roadmap (built) updated" << std::endl;

  // Test graph pruning
  const std::unordered_set<int64_t> nodes_to_prune = {10, 20, 30, 40, 50, 60};
  const auto serial_pruned_roadmap
      = grown_roadmap.MakePrunedCopy(nodes_to_prune, false);
  ASSERT_TRUE(serial_pruned_roadmap.CheckGraphLinkage());
  const auto parallel_pruned_roadmap
      = grown_roadmap.MakePrunedCopy(nodes_to_prune, true);
  ASSERT_TRUE(parallel_pruned_roadmap.CheckGraphLinkage());

  // Helpers for waypoint de/serialization
  const serialization::Serializer<Waypoint> serialize_waypoint_fn
      = [] (const Waypoint& wp, std::vector<uint8_t>& serialization_buffer)
  {
    return serialization::SerializePair<ssize_t, ssize_t>(
        wp, serialization_buffer, serialization::SerializeMemcpyable<ssize_t>,
        serialization::SerializeMemcpyable<ssize_t>);
  };
  const serialization::Deserializer<Waypoint> deserialize_waypoint_fn
      = [] (const std::vector<uint8_t>& deserialization_buffer,
            const uint64_t starting_offset)
  {
    return serialization::DeserializePair<ssize_t, ssize_t>(
        deserialization_buffer, starting_offset,
        serialization::DeserializeMemcpyable<ssize_t>,
        serialization::DeserializeMemcpyable<ssize_t>);
  };

  // Serialize and save roadmaps to file
  std::vector<uint8_t> grown_roadmap_buffer;
  simple_graph::Graph<Waypoint>::Serialize(
      grown_roadmap, grown_roadmap_buffer, serialize_waypoint_fn);
  const std::string grown_roadmap_file =
      "/tmp/temp_planning_test_grown_roadmap.rmp";
  zlib_helpers::CompressAndWriteToFile(
      grown_roadmap_buffer, grown_roadmap_file);

  std::vector<uint8_t> built_roadmap_buffer;
  simple_graph::Graph<Waypoint>::Serialize(
      built_roadmap, built_roadmap_buffer, serialize_waypoint_fn);
  const std::string built_roadmap_file =
      "/tmp/temp_planning_test_built_roadmap.rmp";
  zlib_helpers::CompressAndWriteToFile(
      built_roadmap_buffer, built_roadmap_file);

  // Load from file and deserialize roadmaps
  const std::vector<uint8_t> load_grown_roadmap_buffer =
      zlib_helpers::LoadFromFileAndDecompress(grown_roadmap_file);
  ASSERT_EQ(grown_roadmap_buffer.size(), load_grown_roadmap_buffer.size());
  const auto loaded_grown =
      simple_graph::Graph<Waypoint>::Deserialize(
          load_grown_roadmap_buffer, 0, deserialize_waypoint_fn);
  const simple_graph::Graph<Waypoint>& loaded_grown_roadmap =
      loaded_grown.Value();
  ASSERT_EQ(loaded_grown.BytesRead(), load_grown_roadmap_buffer.size());

  const std::vector<uint8_t> load_built_roadmap_buffer =
      zlib_helpers::LoadFromFileAndDecompress(built_roadmap_file);
  ASSERT_EQ(built_roadmap_buffer.size(), load_built_roadmap_buffer.size());
  const auto loaded_built =
      simple_graph::Graph<Waypoint>::Deserialize(
          load_built_roadmap_buffer, 0, deserialize_waypoint_fn);
  const simple_graph::Graph<Waypoint>& loaded_built_roadmap =
      loaded_built.Value();
  ASSERT_EQ(loaded_built.BytesRead(), load_built_roadmap_buffer.size());

  // Check that loaded roadmaps match
  const auto check_roadmap_loading = [&](
      const simple_graph::Graph<Waypoint>& original_roadmap,
      const simple_graph::Graph<Waypoint>& loaded_roadmap)
  {
    ASSERT_EQ(original_roadmap.Size(), loaded_roadmap.Size());
    for (int64_t idx = 0; idx < original_roadmap.Size(); idx++)
    {
      const auto& old_node = original_roadmap.GetNodeImmutable(idx);
      const auto& new_node = loaded_roadmap.GetNodeImmutable(idx);
      const Waypoint& old_waypoint = old_node.GetValueImmutable();
      const Waypoint& new_waypoint = new_node.GetValueImmutable();
      ASSERT_EQ(old_waypoint.first, new_waypoint.first);
      ASSERT_EQ(old_waypoint.second, new_waypoint.second);
      const auto& old_in_edges = old_node.GetInEdgesImmutable();
      const auto& new_in_edges = new_node.GetInEdgesImmutable();
      ASSERT_EQ(old_in_edges.size(), new_in_edges.size());
      for (size_t edx = 0; edx < new_in_edges.size(); edx++)
      {
        const auto& old_edge = old_in_edges.at(edx);
        const auto& new_edge = new_in_edges.at(edx);
        ASSERT_EQ(old_edge.GetFromIndex(), new_edge.GetFromIndex());
        ASSERT_EQ(old_edge.GetToIndex(), new_edge.GetToIndex());
      }
      const auto& old_out_edges = old_node.GetOutEdgesImmutable();
      const auto& new_out_edges = new_node.GetOutEdgesImmutable();
      ASSERT_EQ(old_out_edges.size(), new_out_edges.size());
      for (size_t edx = 0; edx < new_out_edges.size(); edx++)
      {
        const auto& old_edge = old_out_edges.at(edx);
        const auto& new_edge = new_out_edges.at(edx);
        ASSERT_EQ(old_edge.GetFromIndex(), new_edge.GetFromIndex());
        ASSERT_EQ(old_edge.GetToIndex(), new_edge.GetToIndex());
      }
    }
  };

  check_roadmap_loading(grown_roadmap, loaded_grown_roadmap);
  check_roadmap_loading(built_roadmap, loaded_built_roadmap);

  std::cout << "Loaded Roadmap (grown)" << std::endl;
  DrawRoadmap(test_env, loaded_grown_roadmap);
  std::cout << "Loaded Roadmap (built)" << std::endl;
  DrawRoadmap(test_env, loaded_built_roadmap);

  // Run planning tests
  for (size_t sdx = 0; sdx < keypoints.size(); sdx++)
  {
    for (size_t gdx = 0; gdx < keypoints.size(); gdx++)
    {
      if (sdx != gdx)
      {
        // Get start & goal waypoints
        const Waypoint& start = keypoints.at(sdx);
        const Waypoint& goal = keypoints.at(gdx);

        // Plan with PRM (grown)
        std::cout << "PRM (grown) Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        const auto grown_path =
            simple_prm_planner::QueryPath<Waypoint, WaypointVector>(
                {start}, {goal}, loaded_grown_roadmap, WaypointDistance,
                check_edge_validity_fn, K, false, true, false, true).Path();
        check_plan(test_env, {start}, {goal}, grown_path);

        // Plan with Lazy-PRM (grown)
        std::cout << "Lazy-PRM (grown) Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        const auto lazy_grown_path =
            simple_prm_planner::LazyQueryPath<Waypoint, WaypointVector>(
                {start}, {goal}, loaded_grown_roadmap, WaypointDistance,
                check_edge_validity_fn, K, false, true, false, true).Path();
        check_plan(test_env, {start}, {goal}, lazy_grown_path);

        // Plan with PRM (built)
        std::cout << "PRM (built) Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        const auto built_path =
            simple_prm_planner::QueryPath<Waypoint, WaypointVector>(
                {start}, {goal}, loaded_built_roadmap, WaypointDistance,
                check_edge_validity_fn, K, false, true, false, true).Path();
        check_plan(test_env, {start}, {goal}, built_path);

        // Plan with Lazy-PRM (built)
        std::cout << "Lazy-PRM (built) Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        const auto lazy_built_path =
            simple_prm_planner::LazyQueryPath<Waypoint, WaypointVector>(
                {start}, {goal}, loaded_built_roadmap, WaypointDistance,
                check_edge_validity_fn, K, false, true, false, true).Path();
        check_plan(test_env, {start}, {goal}, lazy_built_path);

        // Plan with A*
        std::cout << "A* Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        const auto astar_path
            = simple_astar_search::PerformAstarSearch<
                Waypoint, WaypointVector, WaypointHasher, WaypointEqualer>(
                    start, goal, GenerateAllPossible8ConnectedChildren,
                    check_edge_validity_fn, WaypointDistance, WaypointDistance,
                    true).Path();
        check_plan(test_env, {start}, {goal}, astar_path);

        // Query-specific RRT helpers
        const auto rrt_sample_fn
            = simple_rrt_planner::MakeStateAndGoalsSamplingFunction<Waypoint>(
                state_sampling_fn, {goal}, rrt_goal_bias, uniform_unit_real_fn);
        const simple_rrt_planner::RRTCheckGoalReachedFunction<Waypoint>
            rrt_goal_reached_fn = [&] (const Waypoint& state)
        {
          return WaypointsEqual(goal, state);
        };

        // Plan with RRT-Extend
        std::cout << "RRT-Extend Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        WaypointPlannerTree rrt_extend_tree;
        rrt_extend_tree.AddNode(start);

        const auto rrt_extend_path
            = simple_rrt_planner::RRTPlanSinglePath<
                Waypoint, WaypointPlannerTree, Waypoint, WaypointVector>(
                    rrt_extend_tree, rrt_sample_fn, rrt_nearest_neighbors_fn,
                    rrt_extend_fn, {}, rrt_goal_reached_fn, {},
                    simple_rrt_planner
                        ::MakeRRTTimeoutTerminationFunction(rrt_timeout))
                        .Path();
        check_plan(test_env, {start}, {goal}, rrt_extend_path);

        // Plan with RRT-Connect
        std::cout << "RRT-Connect Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        WaypointPlannerTree rrt_connect_tree;
        rrt_connect_tree.AddNode(start);

        const auto rrt_connect_path
            = simple_rrt_planner::RRTPlanSinglePath<
                Waypoint, WaypointPlannerTree, Waypoint, WaypointVector>(
                    rrt_connect_tree, rrt_sample_fn, rrt_nearest_neighbors_fn,
                    rrt_connect_fn, {}, rrt_goal_reached_fn, {},
                    simple_rrt_planner
                        ::MakeRRTTimeoutTerminationFunction(rrt_timeout))
                        .Path();
        check_plan(test_env, {start}, {goal}, rrt_connect_path);

        // Plan with BiRRT-Extend
        std::cout << "BiRRT-Extend Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        WaypointPlannerTree birrt_extend_start_tree;
        birrt_extend_start_tree.AddNode(start);
        WaypointPlannerTree birrt_extend_goal_tree;
        birrt_extend_goal_tree.AddNode(goal);

        const auto birrt_extent_path
            = simple_rrt_planner::BiRRTPlanSinglePath<
                Waypoint, WaypointPlannerTree, WaypointVector>(
                    birrt_extend_start_tree, birrt_extend_goal_tree,
                    select_sample_type_fn, state_sampling_fn, tree_sampling_fn,
                    birrt_nearest_neighbors_fn, birrt_extend_fn, {},
                    birrt_states_connected_fn, {}, select_active_tree_fn,
                    simple_rrt_planner
                        ::MakeBiRRTTimeoutTerminationFunction(rrt_timeout))
                .Path();
        check_plan(test_env, {start}, {goal}, birrt_extent_path);

        // Plan with BiRRT-Connect
        std::cout << "BiRRT-Connect Path (" << print::Print(start) << " to "
                  << print::Print(goal) << ")" << std::endl;
        WaypointPlannerTree birrt_connect_start_tree;
        birrt_connect_start_tree.AddNode(start);
        WaypointPlannerTree birrt_connect_goal_tree;
        birrt_connect_goal_tree.AddNode(goal);

        const auto birrt_connect_path
            = simple_rrt_planner::BiRRTPlanSinglePath<
                Waypoint, WaypointPlannerTree, WaypointVector>(
                    birrt_connect_start_tree, birrt_connect_goal_tree,
                    select_sample_type_fn, state_sampling_fn, tree_sampling_fn,
                    birrt_nearest_neighbors_fn, birrt_connect_fn, {},
                    birrt_states_connected_fn, {}, select_active_tree_fn,
                    simple_rrt_planner
                        ::MakeBiRRTTimeoutTerminationFunction(rrt_timeout))
                .Path();
        check_plan(test_env, {start}, {goal}, birrt_connect_path);
      }
    }
  }

  // Plan with multi-start/multi-goal PRM
  const WaypointVector starts = {keypoints.at(0), keypoints.at(1)};
  const WaypointVector goals = {keypoints.at(2), keypoints.at(3)};
  std::cout << "Multi start/goal PRM (grown) Path (" << print::Print(starts)
            << " to " << print::Print(goals) << ")" << std::endl;
  const auto grown_multi_path = simple_prm_planner::QueryPath<Waypoint>(
      starts, goals, loaded_grown_roadmap, WaypointDistance,
      check_edge_validity_fn, K, false, true, false).Path();
  check_plan(test_env, starts, goals, grown_multi_path);
  std::cout << "Multi start/goal PRM (built) Path (" << print::Print(starts)
            << " to " << print::Print(goals) << ")" << std::endl;
  const auto built_multi_path = simple_prm_planner::QueryPath<Waypoint>(
      starts, goals, loaded_built_roadmap, WaypointDistance,
      check_edge_validity_fn, K, false, true, false).Path();
  check_plan(test_env, starts, goals, built_multi_path);

  // Plan with multi-start/multi-goal BiRRT-Connect
  std::cout << "Multi start/goal BiRRT-Connect Path (" << print::Print(starts)
            << " to " << print::Print(goals) << ")" << std::endl;
  WaypointPlannerTree birrt_connect_start_tree;
  for (const Waypoint& start : starts)
  {
    birrt_connect_start_tree.AddNode(start);
  }
  WaypointPlannerTree birrt_connect_goal_tree;
  for (const Waypoint& goal : goals)
  {
    birrt_connect_goal_tree.AddNode(goal);
  }

  const auto birrt_connect_path
      = simple_rrt_planner::BiRRTPlanSinglePath<
          Waypoint, WaypointPlannerTree, WaypointVector>(
              birrt_connect_start_tree, birrt_connect_goal_tree,
              select_sample_type_fn, state_sampling_fn, tree_sampling_fn,
              birrt_nearest_neighbors_fn, birrt_connect_fn, {},
              birrt_states_connected_fn, {}, select_active_tree_fn,
              simple_rrt_planner
                  ::MakeBiRRTTimeoutTerminationFunction(rrt_timeout))
          .Path();
  check_plan(test_env, starts, goals, birrt_connect_path);

  // Use one of the trees to check tree serialization & deserialization
  std::vector<uint8_t> tree_serialization_buffer;
  const uint64_t bytes_written = WaypointPlannerTree::Serialize(
      birrt_connect_start_tree, tree_serialization_buffer,
      serialize_waypoint_fn);
  ASSERT_EQ(
      bytes_written, static_cast<uint64_t>(tree_serialization_buffer.size()));

  const auto deserialized_tree = WaypointPlannerTree::Deserialize(
      tree_serialization_buffer, 0, deserialize_waypoint_fn);
  ASSERT_EQ(bytes_written, deserialized_tree.BytesRead());

  const WaypointPlannerTree& loaded_tree = deserialized_tree.Value();
  ASSERT_EQ(birrt_connect_start_tree.Size(), loaded_tree.Size());

  for (int64_t index = 0; index < loaded_tree.Size(); index++)
  {
    const auto& old_node = birrt_connect_start_tree.GetNodeImmutable(index);
    const auto& loaded_node = loaded_tree.GetNodeImmutable(index);
    ASSERT_EQ(old_node.GetParentIndex(), loaded_node.GetParentIndex());
    ASSERT_TRUE(WaypointsEqual(
        old_node.GetValueImmutable(), loaded_node.GetValueImmutable()));
    ASSERT_EQ(old_node.GetChildIndices().size(),
              loaded_node.GetChildIndices().size());

    for (size_t child = 0; child < old_node.GetChildIndices().size(); child++)
    {
      ASSERT_EQ(old_node.GetChildIndices().at(child),
                loaded_node.GetChildIndices().at(child));
    }
  }
}
}  // namespace planning_test
}  // namespace common_robotics_utilities

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
