#pragma once

/* ORIGINAL LICENSE TEXT:
 *
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <iostream>
#include <functional>
#include <Eigen/Geometry>
#include <vector>
#include <chrono>
#include <list>

namespace common_robotics_utilities
{
namespace time_optimal_trajectory_parametrization
{
class PathSegment
{
public:

  PathSegment(const double length = 0.0) : length_(length) {}

  virtual ~PathSegment() {}

  double Length() const { return length_; }

  virtual Eigen::VectorXd GetConfig(const double s) const = 0;

  virtual Eigen::VectorXd GetTangent(const double s) const = 0;

  virtual Eigen::VectorXd GetCurvature(const double s) const = 0;

  virtual std::list<double> GetSwitchingPoints() const = 0;

  virtual PathSegment* Clone() const = 0;

  double position;

protected:

  double length_;
};



class Path
{
public:

  Path(const std::list<Eigen::VectorXd>& path,
       const double maxDeviation = 0.0);

  Path(const Path& path);

  ~Path();

  double Length() const;

  Eigen::VectorXd GetConfig(const double s) const;

  Eigen::VectorXd GetTangent(const double s) const;

  Eigen::VectorXd GetCurvature(const double s) const;

  double GetNextSwitchingPoint(const double s, bool& discontinuity) const;

  std::list<std::pair<double, bool>> SwitchingPoints() const;

private:

  PathSegment* GetPathSegment(double& s) const;

  double length_;

  std::list<std::pair<double, bool>> switching_points_;

  std::list<PathSegment*> path_segments_;
};

class Trajectory
{
public:
  // Generates a time-optimal trajectory
  Trajectory(const std::list<Eigen::VectorXd>& waypoints,
             const Eigen::VectorXd& max_velocity,
             const Eigen::VectorXd& max_acceleration,
             const double max_deviation,
             const double timestep);

  Trajectory(const Path &path_,
             const Eigen::VectorXd& max_velocity_,
             const Eigen::VectorXd& max_acceleration_,
             double timestep = 0.001);

  ~Trajectory(void);

  // Returns the optimal duration of the trajectory
  double Duration() const;

  // Return the position vector and velocity vector of the
  // robot for a given point in time within the trajectory.
  // !!! NOT THREAD SAFE - MUTABLE CACHE INSIDE !!!
  std::pair<Eigen::VectorXd, Eigen::VectorXd> GetPositionVelocity(
      const double time) const;

  // Outputs the phase trajectory and the velocity limit curve
  // in 2 files for debugging purposes.
  void OutputPhasePlaneTrajectory() const;

private:

  struct TrajectoryStep
  {
    TrajectoryStep() {}

    TrajectoryStep(double path_pos, double path_vel)
      : path_pos_(path_pos), path_vel_(path_vel) {}

    double path_pos_;
    double path_vel_;
    double time_;
  };

  bool GetNextSwitchingPoint(const double path_pos,
                             TrajectoryStep& next_switching_point,
                             double& before_acceleration,
                             double& after_acceleration);

  bool GetNextAccelerationSwitchingPoint(const double path_pos,
                                         TrajectoryStep& next_switching_point,
                                         double& before_acceleration,
                                         double& after_acceleration);

  bool GetNextVelocitySwitchingPoint(const double path_pos,
                                     TrajectoryStep& next_switching_point,
                                     double& before_acceleration,
                                     double& after_acceleration);

  bool IntegrateForward(std::list<TrajectoryStep>& trajectory,
                        const double acceleration);

  void IntegrateBackward(std::list<TrajectoryStep>& start_trajectory,
                         const double path_pos,
                         const double path_vel,
                         const double acceleration);

  double GetMinMaxPathAcceleration(const double path_position,
                                   const double path_velocity,
                                   const bool max);

  double GetMinMaxPhaseSlope(const double path_position,
                             const double path_velocity,
                             const bool max);

  double GetAccelerationMaxPathVelocity(const double path_pos) const;

  double GetVelocityMaxPathVelocity(const double path_pos) const;

  double GetAccelerationMaxPathVelocityDeriv(const double path_pos);

  double GetVelocityMaxPathVelocityDeriv(const double path_pos);

  std::list<TrajectoryStep>::const_iterator GetTrajectorySegment(
      const double time) const;

  Path path_;
  Eigen::VectorXd max_velocity_;
  Eigen::VectorXd max_acceleration_;
  uint32_t n_ = 0u;
  std::list<TrajectoryStep> trajectory_;

  static const double eps;
  const double time_step_;

  mutable double cached_time_;
  mutable std::list<TrajectoryStep>::const_iterator cached_trajectory_segment_;
};
}  // namespace time_optimal_trajectory_parametrization
}  // namespace common_robotics_utilities
