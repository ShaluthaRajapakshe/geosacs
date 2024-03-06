# F&amp;P Descriptions - Developer's Guide

This document contains information for F&amp;P developers. It's an addition to
the standard README, which is meant to be visible to the public.

## Meshes
When updating the STL files, it is important to place the origins at locations 
where they can easily describe the transformation from that origin to the joint 
origin. In most cases, we aim at having the two coincide.

## Package Structure

As a guideline, each description package should comprise the directories `urdf`, 
`meshes` and (optionally) `config`. The `urdf`-directory contains both the xacro 
files and the expanded URDF files for the most important robot configurations.

When a developer has modified a `xacro` file, it is their responsibility to 
create the depending `urdf`s before committing.

`meshes` and `config` are subdivided into directories, carrying the names of the 
different versions, which may exist for this component (e.g. `p_rob_1U` vs. 
`p_rob_2R`, etc.). `meshes` then contains the required STL files, while `config` 
contains one `config.yaml` file, storing configuration values for this 
component. This way, they don't need to be hard-coded in the `xacro` file.

## Robot-Specific Configuration
`pcare_description` and `lio_description` provide all URDFs for service robots, 
but not all Lios are exactly the same. In order to avoid seperate URDFs for the 
first few Lios, we are using environment variables instead.

Both P-Care and Lio need the environment variable `ROBOT_TYPE` to be set. 
The following ROBOT_TYPE are available: PCAc, LIOa, LIOb, LIOc.

lio_description needs:

`RDSCFG_fp_descriptions_lidar_front_rotation` which specifies the rotation of 
thefront lidar. Default value would be 0, but due to mounting, this cannot be 
applied to all robots.

`RDSCFG_fp_descriptions_lidar_back_rotation` which specifies the rotation of the
back lidar. Default value would be 3.14, but due to mounting, this cannot be 
applied to all robots.

`RDSCFG_fp_descriptions_robot_frame_rotation` which specifies the rotation of 
the robot frame. Lio2001 was mounted differently on its mobile platform than the 
others, so this parameter needs to be different. Default value would be 1.57.

`RDSCFG_fp_descriptions_j1_frame_rotation` which specifies the rotation of the
joint1 frame. Lio2001 was mounted differently on its mobile platform than the 
others, so this parameter needs to be different. Default value would be 0.

These environment variables are set on each robot. To use the URDFs locally, 
please run `export PARAM_NAME=PARAM_VALUE` before running the launch file. 
