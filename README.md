# common_robotics_utilities
Common utility functions and algorithms for robotics work used by ARC &amp; ARM labs and TRI.

## Setup

For convenience of building, `common_robotics_utilities` is a ROS package which
supports [ROS 2 Jazzy](https://docs.ros.org/en/rolling/Installation.html) and
newer ROS 2 distributions. Prior distributions of ROS 2 can be supported by
removing ROS 2 message printing support from
`include/common_robotics_utilities/print.hpp`. Most of this project can be built
entirely without ROS support, see the example of the
[Drake](https://github.com/RobotLocomotion/drake) project which builds this
subset using Bazel for both Linux and macOS platforms.

As provided, `common_robotics_utilities` should be built in a ROS 2 workspace:

```sh
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/calderpg/common_robotics_utilities.git
```

Finally, use [`rosdep`](https://docs.ros.org/independent/api/rosdep/html/)
to ensure all dependencies in the `package.xml` are satisfied:

```sh
cd ~/ws
rosdep install -i -y --from-path src
```

## Building

Use [`colcon`](https://colcon.readthedocs.io/en/released/):

```sh
cd ~/ws
colcon build  # the entire workspace
colcon build --packages-select common_robotics_utilities  # the package only
```

## Testing

Use [`colcon`](https://colcon.readthedocs.io/en/released/):

```sh
cd ~/ws
colcon test --event-handlers=console_direct+  # the entire workspace
colcon test --event-handlers=console_direct+ --packages-select common_robotics_utilities  # the package only
```
