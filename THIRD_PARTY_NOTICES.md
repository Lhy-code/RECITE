# Third-party notices

## cuRobo scene configurations

The `CUROBO_SCENES` definitions in `recite/scenes.py` retain adapted cuRobo-family geometry (`curobo_cage`, `curobo_wall`, `curobo_thin_walls`, and `curobo_pillar_wall`).

Original project: https://github.com/NVlabs/curobo

Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

These scene configurations and their derivatives retain the [NVIDIA License](LICENSES/curobo.txt), including its non-commercial research/evaluation use terms. The first-party RECITE implementation uses the MIT License; the adapted scene configurations retain the NVIDIA terms above. Adaptations include the RM65 scene representation, workspace-scale placements, and the raised/shortened wall in `curobo_wall`.

The cuRobo material distributed here consists of the adapted scene configurations listed above. The upstream project distributes the cuRobo planner, CUDA code, and robot assets.

## MoveIt-style geometric scenes

The `MOVEIT_SCENES` group contains MoveIt-style scene layouts expressed as RECITE boxes. The `moveit_proxy` tag records this geometric scene family. MoveIt software, meshes, and binaries are distributed through their upstream projects.

## NumPy

NumPy is installed separately as a dependency. Its license and notices are provided by the installed NumPy distribution.
