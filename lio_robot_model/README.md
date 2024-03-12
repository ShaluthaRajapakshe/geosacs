![alt text](https://www.fp-robotics.com/wp-content/themes/fprobot/img/fp-robotics_logo.png "F&P Robotics AG")

# F&amp;P Descriptions

This stack contains packages that provide the URDF models of F&amp;P robots for 
use with visualizers and simulators.

## Meshes
These define the geometry of each link's outer shell. They are used for 
collision avoidance and visualization.

## URDF
The files ending with `.urdf` are XML descriptions of robots, which are used for 
visualization, collision avoidance and kinematics. Since these files often 
contain repetitive structures, it would be rather tedious to write and maintain 
them by hand. Xacro files simplify this task.

## Xacro Files
Xacro lets you define XML macros to then autogenerate a URDF file. In this case, 
it can mean that we define our model using constants and function-like macros, 
instead of hard-coding everything.

This means: If something changes in the model, like the length of a part used in 
multiple places, we don't need to go through and redefine the whole model, but 
instead just update the constant it refers to.

To autogenerate the URDF code, run

```
rosrun xacro xacro --inorder -o p_hand.urdf p_hand.urdf.xacro
```

To learn more about URDF and xacro, please refer to 
[this tutorial](http://wiki.ros.org/urdf/Tutorials "URDF tutorials").

## Package Structure
Packages, whose name ends on "`_description`", contain descriptions for robots 
or parts of robots. For instance, `p_rob_description` merely defines the base 
and then includes as macros the motors and links of the arm, as well as the 
gripper, like this:

```xml
  <xacro:p_arm name="p_rob" parent="base_link" version="p_rob_2R">
    <origin xyz="0 0 0.212"/>
  </xacro:p_arm>

  <xacro:p_grip
    name="p_rob"
    parent="p_rob_gripper_interface_link"
    version="p_grip_2F"
    orientation="side"/>
```

`p_arm` and `p_grip` are defined in `p_arm_description` and 
`p_grip_description`, respectively, which are being included in the very 
beginning of `p_rob_description`. A few parameters are parsed to them, such as 
their respective parent link, which they shall be attached to, or the 
configuration, in which the gripper is mounted. This allows for a very modular 
build-up of F&amp;P  components. If a P-Rob is simulated without its gripper, 
simply remove the `p_grip`-xacro in `p_rob_description`. Also, a P-Arm can be 
recycled in the descriptions of F&amp;P's service robot products, where the arm 
is no longer mounted on a standard base.

Finally, material properties (e.g. colors) are stored in `fp_materials` and used 
throughout the description packages.

# Versions
## 0.2.0
- Last Ubuntu 16.04 compatible version
## 0.3.0
- Updated for Ubuntu 20.04 compatibility

