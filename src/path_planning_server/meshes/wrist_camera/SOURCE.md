# Wrist camera mesh sources

The D435i wrist-camera assets in this directory were copied without geometry
modification from `agilexrobotics/mobile_aloha_sim` at commit `f799ae0`.

- `d435_camera_stand.dae`
  - Source:
    `aloha_new_description/meshes/dark_dae/d435_camera_stand.dae`
  - Used by:
    `aloha_new_description/urdf/aloha_tracer2_four_d435_dark.urdf`
  - Source package license: BSD
- `d435.dae`
  - Source: `realsense2_description/meshes/d435.dae`
  - Shared by the D435 and D435i descriptions; the D435i model is selected.
  - Source package license: Apache-2.0

The stand transforms are based on the front-left and front-right wrist
assemblies in `aloha_tracer2_four_d435_dark.urdf`. The D435i body transform,
collision box and inertia are based on
`realsense2_description/urdf/_d435i.urdf.xacro` and its included D435 housing
description. The stand attachment pose is converted to the coordinate
convention of the unmodified `piper_description` wrist link.
