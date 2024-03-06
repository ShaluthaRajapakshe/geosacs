drone-panda
===========
Robot Controller
----------------

*This work is part of a larger project, it is designed to be used with the
[panda-ik](https://github.com/emmanuel-senft/panda_ik) and [panda-drone-msgs](https://github.com/emmanuel-senft/drone-ros-msgs).*

Installation
------------

If not yet installed, start by [installing
ROS](http://wiki.ros.org/ROS/Installation) (tested with ROS Noetic on Ubuntu 20.04).

Dependencies:
- [panda-ik](https://github.com/emmanuel-senft/panda_ik)
- [panda-ros-msgs](https://github.com/emmanuel-senft/panda-ros-msgs)
- [rviz_camera_stream](https://github.com/lucasw/rviz_camera_stream)
- [rviz_lighting](https://github.com/mogumbo/rviz_lighting)
- [eigen3](https://eigen.tuxfamily.org/dox/)
- [teleop_trist_joy](http://wiki.ros.org/teleop_twist_joy)
- [ros-numpy](http://wiki.ros.org/ros_numpy)

Recommend to use [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) to compile.
Then build with:

```
> catkin build panda_ik
> catkin build drone_panda
```

Usage
-----

### Starting the robot controller 
- `roslaunch drone_panda sim.launch`

The system is designed to be used with a gamepad that can be used with ros teleop_twist_joy.
