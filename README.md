# common_robotics_utilities
Common utility functions and algorithms for robotics work used by ARC &amp; ARM labs and TRI.

## Setup

`common_robotics_utilities` is a ROS package.

Thus, it is best to build it within a ROS workspace:

```sh
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/calderpg/common_robotics_utilities.git
```

This package supports [ROS 1 Kinetic+](http://wiki.ros.org/ROS/Installation)
and [ROS 2 Dashing+](https://index.ros.org/doc/ros2/Installation/) distributions.
Make sure to symlink the corresponding `CMakeLists.txt` and `package.xml` files
for the ROS distribution of choice:

*For ROS 1 Kinetic+*
```sh
cd ~/ws/src/common_robotics_utilities
ln -sT CMakeLists.txt.ros1 CMakeLists.txt
ln -sT package.xml.ros1 package.xml
```

*For ROS 2 Dashing+*
```sh
cd ~/ws/src/common_robotics_utilities
ln -sT CMakeLists.txt.ros2 CMakeLists.txt
ln -sT package.xml.ros2 package.xml
```

Finally, use [`rosdep`](https://docs.ros.org/independent/api/rosdep/html/)
to ensure all dependencies in the `package.xml` are satisfied:

```sh
cd ~/ws
rosdep install -i -y --from-path src
```

## Building

Use [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) or
[`colcon`](https://colcon.readthedocs.io/en/released/) accordingly.

*For ROS 1 Kinetic+*
```sh
cd ~/ws
catkin_make  # the entire workspace
catkin_make --pkg common_robotics_utilities  # the package only
```

*For ROS 2 Dashing +*
```sh
cd ~/ws
colcon build  # the entire workspace
colcon build --packages-select common_robotics_utilities  # the package only
```

## Testing

Use [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) or
[`colcon`](https://colcon.readthedocs.io/en/released/) accordingly.

*For ROS 1 Kinetic+*
```sh
cd ~/ws
catkin_make run_tests  # the entire workspace
catkin_make run_tests_common_robotics_utilities  # the package only
```

*For ROS 2 Dashing +*
```sh
cd ~/ws
colcon test --event-handlers=console_direct+  # the entire workspace
colcon test --event-handlers=console_direct+ --packages-select common_robotics_utilities  # the package only
```
