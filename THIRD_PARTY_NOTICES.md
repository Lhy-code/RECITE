# Third-party notices

## cuRobo scene configurations

The `CUROBO_SCENES` definitions in `recite/scenes.py` retain adapted cuRobo-family geometry (`curobo_cage`, `curobo_wall`, `curobo_thin_walls`, and `curobo_pillar_wall`).

Original project: https://github.com/NVlabs/curobo

Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

These scene configurations and their derivatives remain subject to the [NVIDIA License](LICENSES/curobo.txt), including its non-commercial research/evaluation use limitation. The MIT license for the RECITE implementation does not replace these terms. Adaptations include the RM65 scene representation, workspace-scale placements, and the raised/shortened wall in `curobo_wall`.

The cuRobo planner, CUDA code, and robot assets are not bundled.

## MoveIt-style proxy scenes

The `MOVEIT_SCENES` group contains geometric proxy scenes expressed as RECITE boxes. The `moveit_proxy` tag records this scene family; no MoveIt planner implementation, meshes, or binaries are bundled.

## NumPy

NumPy is installed separately as a dependency and is not redistributed here. Its license and notices are provided by the installed NumPy distribution.
