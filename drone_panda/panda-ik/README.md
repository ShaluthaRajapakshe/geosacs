panda-ik
========
Inverse kinematic for panda and a drone camera
----------------

*This work is part of a larger project, it is designed to be used with the
[drone-panda](https://github.com/emmanuel-senft/drone-panda).*

Installation
------------

If not yet installed, start by [installing
ROS](http://wiki.ros.org/ROS/Installation) (tested with ROS Noetic, but
other versions might work as well).

Dependencies:
- [rust](https://www.rust-lang.org/tools/install)
- [eigen3](https://eigen.tuxfamily.org/dox/)

Recommend to use [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) to compile, it automatically compile the rust code too.
Then build with:

```
> catkin build panda_ik
```

Rust build errors can be found in the `panda_ik/src/panda_ik_rust-stamp/panda_ik_rust-build-err.log` file.

Usage
-----

### Starting the robot controller 
- `roslaunch drone_panda all.launch`

Send the desired robot end effector position, e.g.:

`pose_pub = rospy.Publisher("/panda_ik/input", PoseStamped, queue_size=5)`

`p = PoseStamped()`

`p.header.frame_id="panda_gripper_joint"`

`p.pose.position.x = .5`

`p.pose.position.y = 0`

`p.pose.position.z=.3`

`p.pose.orientation.w=1`

`pose_pub.publish(p)`
