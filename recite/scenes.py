"""Original 25-template static scene generation and geometry rules.

The geometry screen checks workspace bounds and obstacle clearances before
route planning establishes a collision-free robot path.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class Obstacle:
    name: str
    center: tuple          # (x, y, z)
    size: tuple            # (sx, sy, sz) full extents
    color: str = "#5588BB"
    alpha: float = 0.25
    is_table: bool = False  # 大地面不参与碰撞检测
    quaternion: tuple = (1.0, 0.0, 0.0, 0.0)  # [qw, qx, qy, qz] — OBB 旋转

@dataclass
class Scene:
    name: str
    goal_pos: tuple        # (x, y, z)
    obstacles: list        # list[Obstacle]
    source: str = ""       # rm65_realistic / moveit_proxy / curobo_builtin / claude_new
    category: str = "base" # base / variant
    parent: str = ""
    variant_id: int = 0


# ════════════════════════════════════════════════
#  RM65 参数 (基座在原点, z-up)
# ════════════════════════════════════════════════
BASE_POS = np.array([0.0, 0.0, 0.245])
BASE_RADIUS = 0.08
ARM_SPHERES_HOME = [
    (0.00, 0.00, 0.245),
    (0.00, 0.00, 0.355),
    (0.12, 0.00, 0.395),
    (0.24, 0.00, 0.395),
    (0.31, 0.00, 0.395),
    (0.35, 0.00, 0.395),
]
ARM_SPHERE_RADII = [0.06, 0.05, 0.04, 0.035, 0.03, 0.025]
# 工作空间边界 (保守)
WS_MIN = np.array([0.12, -0.30, 0.05])
WS_MAX = np.array([0.62, 0.30, 0.55])
GOAL_MIN = np.array([0.15, -0.25, 0.15])
GOAL_MAX = np.array([0.58, 0.25, 0.50])
MAX_REACH = 0.55   # 目标到 base 的最大距离
OBS_MAX_DIST = 0.80  # 障碍到 base 的最大距离


# ════════════════════════════════════════════════
#  A. RM65 Realistic Scenes (5, shared between Codex & Claude)
# ════════════════════════════════════════════════
RM65_SCENES = {
    "rm65_corridor": Scene(
        name="rm65_corridor",
        goal_pos=(0.40, 0.12, 0.35),
        obstacles=[
            Obstacle("ceiling", (0.49, 0.0, 0.44), (0.28, 0.44, 0.024)),
            Obstacle("floor", (0.49, 0.0, 0.20), (0.28, 0.44, 0.024)),
        ],
        source="rm65_realistic",
    ),
    "rm65_shelf_reach": Scene(
        name="rm65_shelf_reach",
        goal_pos=(0.40, 0.12, 0.35),
        obstacles=[
            Obstacle("shelf_top", (0.49, 0.0, 0.44), (0.30, 0.44, 0.024)),
        ],
        source="rm65_realistic",
    ),
    "rm65_drawer_wall": Scene(
        name="rm65_drawer_wall",
        goal_pos=(0.38, 0.12, 0.35),
        obstacles=[
            Obstacle("side_wall_+Y", (0.44, 0.22, 0.32), (0.24, 0.024, 0.28)),
            Obstacle("side_wall_-Y", (0.44, -0.22, 0.32), (0.24, 0.024, 0.28)),
            Obstacle("back_wall", (0.58, 0.0, 0.32), (0.024, 0.44, 0.28)),
            Obstacle("floor", (0.46, 0.0, 0.20), (0.28, 0.36, 0.024)),
            Obstacle("top", (0.46, 0.0, 0.44), (0.28, 0.36, 0.024)),
        ],
        source="rm65_realistic",
    ),
    "rm65_cubby_shelf": Scene(
        name="rm65_cubby_shelf",
        goal_pos=(0.50, 0.0, 0.30),
        obstacles=[
            Obstacle("wall_+Y", (0.55, 0.18, 0.33), (0.24, 0.02, 0.36)),
            Obstacle("wall_-Y", (0.55, -0.18, 0.33), (0.24, 0.02, 0.36)),
            Obstacle("back_wall", (0.67, 0.0, 0.33), (0.02, 0.36, 0.36)),
            Obstacle("floor", (0.55, 0.0, 0.15), (0.24, 0.36, 0.02)),
        ],
        source="rm65_realistic",
    ),
    "rm65_cabinet_reach": Scene(
        name="rm65_cabinet_reach",
        goal_pos=(0.38, 0.12, 0.35),
        obstacles=[
            Obstacle("top_shelf", (0.48, 0.0, 0.44), (0.24, 0.32, 0.024)),
            Obstacle("bottom_shelf", (0.48, 0.0, 0.20), (0.24, 0.32, 0.024)),
            Obstacle("back_wall", (0.62, 0.0, 0.32), (0.024, 0.32, 0.28)),
        ],
        source="rm65_realistic",
    ),
}

# ════════════════════════════════════════════════
#  B. MoveIt-style Geometric Scenes (6, Codex collection)
# ════════════════════════════════════════════════
MOVEIT_SCENES = {
    "moveit_bookshelf_small": Scene(
        name="moveit_bookshelf_small",
        goal_pos=(0.42, 0.04, 0.28),
        obstacles=[
            Obstacle("left_panel", (0.46, 0.14, 0.29), (0.20, 0.02, 0.32), "#CC7722"),
            Obstacle("right_panel", (0.46, -0.14, 0.29), (0.20, 0.02, 0.32), "#CC7722"),
            Obstacle("back_panel", (0.56, 0.0, 0.29), (0.02, 0.28, 0.32), "#4488DD"),
            Obstacle("top_panel", (0.46, 0.0, 0.43), (0.20, 0.28, 0.02), "#BB5599"),
        ],
        source="moveit_proxy",
    ),
    "moveit_bookshelf_tall": Scene(
        name="moveit_bookshelf_tall",
        goal_pos=(0.46, 0.04, 0.40),
        obstacles=[
            Obstacle("left_panel", (0.50, 0.16, 0.42), (0.24, 0.02, 0.56), "#CC7722"),
            Obstacle("right_panel", (0.50, -0.16, 0.42), (0.24, 0.02, 0.56), "#CC7722"),
            Obstacle("back_panel", (0.62, 0.0, 0.42), (0.02, 0.32, 0.56), "#4488DD"),
            Obstacle("mid_shelf", (0.50, 0.0, 0.34), (0.24, 0.32, 0.02), "#55BB44"),
        ],
        source="moveit_proxy",
    ),
    "moveit_bookshelf_thin": Scene(
        name="moveit_bookshelf_thin",
        goal_pos=(0.40, 0.0, 0.32),
        obstacles=[
            Obstacle("left_panel", (0.42, 0.10, 0.32), (0.26, 0.016, 0.36), "#CC7722"),
            Obstacle("right_panel", (0.42, -0.10, 0.32), (0.26, 0.016, 0.36), "#CC7722"),
            Obstacle("back_panel", (0.55, 0.0, 0.32), (0.016, 0.20, 0.36), "#4488DD"),
            Obstacle("top_panel", (0.42, 0.0, 0.46), (0.26, 0.20, 0.016), "#BB5599"),
            Obstacle("bottom_panel", (0.42, 0.0, 0.18), (0.26, 0.20, 0.016), "#55BB44"),
        ],
        source="moveit_proxy",
    ),
    "moveit_cage": Scene(
        name="moveit_cage",
        goal_pos=(0.52, 0.0, 0.20),
        obstacles=[
            Obstacle("bar_1", (0.36, 0.12, 0.24), (0.02, 0.02, 0.48), "#4488DD"),
            Obstacle("bar_2", (0.36, -0.12, 0.24), (0.02, 0.02, 0.48), "#4488DD"),
            Obstacle("bar_3", (0.58, 0.14, 0.24), (0.02, 0.02, 0.48), "#4488DD"),
            Obstacle("bar_4", (0.58, -0.14, 0.24), (0.02, 0.02, 0.48), "#4488DD"),
            Obstacle("upper_link", (0.47, 0.0, 0.42), (0.24, 0.28, 0.02), "#BB5599"),
            Obstacle("lower_link", (0.47, 0.0, 0.06), (0.24, 0.28, 0.02), "#55BB44"),
        ],
        source="moveit_proxy",
    ),
    "moveit_table_bars": Scene(
        name="moveit_table_bars",
        goal_pos=(0.38, 0.08, 0.30),
        obstacles=[
            Obstacle("table", (0.42, 0.0, -0.02), (0.80, 0.80, 0.04), "#888888", 0.12, is_table=True),
            Obstacle("bar_left", (0.34, 0.14, 0.22), (0.02, 0.02, 0.44), "#CC7722"),
            Obstacle("bar_mid", (0.42, 0.0, 0.22), (0.02, 0.02, 0.44), "#CC7722"),
            Obstacle("bar_right", (0.50, -0.14, 0.22), (0.02, 0.02, 0.44), "#CC7722"),
        ],
        source="moveit_proxy",
    ),
    "moveit_kitchen": Scene(
        name="moveit_kitchen",
        goal_pos=(0.49, 0.18, 0.30),
        obstacles=[
            Obstacle("counter_back", (0.68, 0.0, 0.32), (0.02, 0.80, 0.36), "#4488DD"),
            Obstacle("counter_left", (0.50, 0.26, 0.24), (0.36, 0.02, 0.24), "#CC7722"),
            Obstacle("upper_cabinet", (0.52, 0.08, 0.48), (0.28, 0.40, 0.02), "#BB5599"),
            Obstacle("countertop", (0.50, 0.0, 0.18), (0.36, 0.60, 0.02), "#55BB44"),
            Obstacle("side_block", (0.38, -0.18, 0.28), (0.12, 0.16, 0.28), "#6699AA"),
        ],
        source="moveit_proxy",
    ),
}

# ════════════════════════════════════════════════
#  C. cuRobo Builtin Scenes (4, Codex collection)
#     Adapted from Franka Panda layouts; validate_scene applies RM65 geometry rules.
#     Adapted configurations: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES.
#     Subject to LICENSES/curobo.txt, including non-commercial research/evaluation use.
# ════════════════════════════════════════════════
CUROBO_SCENES = {
    "curobo_cage": Scene(
        name="curobo_cage",
        goal_pos=(0.46, 0.0, 0.28),
        obstacles=[
            Obstacle("cage_bar_fl", (0.32, 0.12, 0.28), (0.02, 0.02, 0.36), "#8866AA"),
            Obstacle("cage_bar_fr", (0.32, -0.12, 0.28), (0.02, 0.02, 0.36), "#8866AA"),
            Obstacle("cage_bar_bl", (0.56, 0.12, 0.28), (0.02, 0.02, 0.36), "#8866AA"),
            Obstacle("cage_bar_br", (0.56, -0.12, 0.28), (0.02, 0.02, 0.36), "#8866AA"),
            Obstacle("cage_top", (0.44, 0.0, 0.44), (0.26, 0.26, 0.02), "#8866AA"),
            Obstacle("cage_bottom", (0.44, 0.0, 0.12), (0.26, 0.26, 0.02), "#8866AA"),
        ],
        source="curobo_builtin",
    ),
    "curobo_wall": Scene(
        name="curobo_wall",
        goal_pos=(0.50, 0.00, 0.30),
        obstacles=[
            Obstacle("table", (0.0, 0.0, -0.1), (2.2, 2.2, 0.2), "#888888", 0.10, is_table=True),
            Obstacle("wall", (0.30, 0.0, 0.50), (0.05, 0.4, 0.20), "#DD4444"),  # raised+shortened for HOME_Q clearance
        ],
        source="curobo_builtin",
    ),
    "curobo_thin_walls": Scene(
        name="curobo_thin_walls",
        goal_pos=(0.34, 0.00, 0.34),
        obstacles=[
            Obstacle("table", (0.0, 0.0, -0.1), (2.2, 2.2, 0.2), "#888888", 0.10, is_table=True),
            Obstacle("wall_1", (0.4, -0.1, 0.3), (0.05, 0.01, 1.5), "#DD4444"),
            Obstacle("wall_2", (0.0, 0.4, 0.3), (0.05, 0.01, 1.5), "#DD4444"),
            Obstacle("wall_3", (0.0, -0.4, 0.3), (0.05, 0.01, 1.5), "#DD4444"),
        ],
        source="curobo_builtin",
    ),
    "curobo_pillar_wall": Scene(
        name="curobo_pillar_wall",
        goal_pos=(0.46, 0.12, 0.30),
        obstacles=[
            Obstacle("table", (0.0, 0.0, -0.1), (2.2, 2.2, 0.2), "#888888", 0.10, is_table=True),
            Obstacle("pillar", (0.4, 0.0, 0.3), (0.05, 0.05, 0.6), "#DD4444"),
            Obstacle("wall_back", (-0.6, 0.0, 0.3), (0.05, 0.05, 0.6), "#DD4444"),
        ],
        source="curobo_builtin",
    ),
}

# ════════════════════════════════════════════════
#  D. Claude New Topologies (10, Claude collection)
# ════════════════════════════════════════════════
CLAUDE_NEW_SCENES = {
    "rm65_l_bend": Scene(
        name="rm65_l_bend",
        goal_pos=(0.45, 0.0, 0.32),
        obstacles=[
            Obstacle("l_horiz", (0.42, 0.12, 0.32), (0.22, 0.02, 0.20)),
            Obstacle("l_vert", (0.32, 0.0, 0.32), (0.02, 0.24, 0.20)),
        ],
        source="claude_new",
    ),
    "rm65_dual_pillar": Scene(
        name="rm65_dual_pillar",
        goal_pos=(0.40, 0.18, 0.35),
        obstacles=[
            Obstacle("pillar_a", (0.32, -0.06, 0.32), (0.06, 0.06, 0.30)),
            Obstacle("pillar_b", (0.32, 0.10, 0.32), (0.06, 0.06, 0.30)),
        ],
        source="claude_new",
    ),
    "rm65_narrow_slit": Scene(
        name="rm65_narrow_slit",
        goal_pos=(0.42, 0.0, 0.35),
        obstacles=[
            Obstacle("slit_wall_pos", (0.34, 0.08, 0.32), (0.24, 0.02, 0.32)),
            Obstacle("slit_wall_neg", (0.34, -0.08, 0.32), (0.24, 0.02, 0.32)),
        ],
        source="claude_new",
    ),
    "rm65_table_fence": Scene(
        name="rm65_table_fence",
        goal_pos=(0.40, 0.0, 0.28),
        obstacles=[
            Obstacle("table_top", (0.38, 0.0, 0.22), (0.36, 0.50, 0.02)),
            Obstacle("fence_left", (0.20, -0.18, 0.30), (0.02, 0.02, 0.14)),
            Obstacle("fence_right", (0.20, 0.18, 0.30), (0.02, 0.02, 0.14)),
            Obstacle("fence_back", (0.56, 0.0, 0.30), (0.02, 0.36, 0.14)),
        ],
        source="claude_new",
    ),
    "rm65_multi_shelf": Scene(
        name="rm65_multi_shelf",
        goal_pos=(0.41, 0.05, 0.30),
        obstacles=[
            Obstacle("shelf_low", (0.50, 0.0, 0.20), (0.28, 0.40, 0.02)),
            Obstacle("shelf_mid", (0.50, 0.0, 0.40), (0.28, 0.40, 0.02)),
            Obstacle("shelf_high", (0.50, 0.0, 0.48), (0.28, 0.40, 0.02)),
        ],
        source="claude_new",
    ),
    "rm65_open_field": Scene(
        name="rm65_open_field",
        goal_pos=(0.40, 0.15, 0.30),
        obstacles=[
            Obstacle("small_block", (0.30, 0.0, 0.25), (0.06, 0.06, 0.06)),
        ],
        source="claude_new",
    ),
    "rm65_tunnel": Scene(
        name="rm65_tunnel",
        goal_pos=(0.45, 0.0, 0.32),
        obstacles=[
            Obstacle("tunnel_top", (0.42, 0.0, 0.46), (0.30, 0.20, 0.02)),     # x: 0.38→0.42 avoid HOME_Q
            Obstacle("tunnel_bottom", (0.42, 0.0, 0.18), (0.30, 0.20, 0.02)),  # x: 0.38→0.42
            Obstacle("tunnel_left", (0.42, -0.10, 0.32), (0.30, 0.02, 0.26)),  # x: 0.38→0.42
        ],
        source="claude_new",
    ),
    "rm65_overhead_bar": Scene(
        name="rm65_overhead_bar",
        goal_pos=(0.38, 0.12, 0.28),
        obstacles=[
            Obstacle("bar_horizontal", (0.36, 0.0, 0.38), (0.08, 0.40, 0.04)),
        ],
        source="claude_new",
    ),
    "rm65_triple_pillar": Scene(
        name="rm65_triple_pillar",
        goal_pos=(0.48, 0.0, 0.35),
        obstacles=[
            Obstacle("pillar_1", (0.20, -0.10, 0.30), (0.05, 0.05, 0.28)),
            Obstacle("pillar_2", (0.36, 0.18, 0.30), (0.05, 0.05, 0.28)),
            Obstacle("pillar_3", (0.54, -0.05, 0.30), (0.05, 0.05, 0.28)),
        ],
        source="claude_new",
    ),
    "rm65_slot_entry": Scene(
        name="rm65_slot_entry",
        goal_pos=(0.50, 0.0, 0.32),
        obstacles=[
            Obstacle("slot_top", (0.44, 0.0, 0.42), (0.24, 0.30, 0.02)),
            Obstacle("slot_bottom", (0.44, 0.0, 0.22), (0.24, 0.30, 0.02)),
            Obstacle("slot_left", (0.44, -0.15, 0.32), (0.24, 0.02, 0.18)),
            Obstacle("slot_right", (0.44, 0.15, 0.32), (0.24, 0.02, 0.18)),
            Obstacle("slot_back", (0.56, 0.0, 0.32), (0.02, 0.30, 0.18)),
        ],
        source="claude_new",
    ),
}


# ════════════════════════════════════════════════
#  汇总
# ════════════════════════════════════════════════
ALL_BASE_SCENES: dict[str, Scene] = {}
ALL_BASE_SCENES.update(RM65_SCENES)       # 5
ALL_BASE_SCENES.update(MOVEIT_SCENES)     # 6
ALL_BASE_SCENES.update(CUROBO_SCENES)     # 4
ALL_BASE_SCENES.update(CLAUDE_NEW_SCENES) # 10
# Total: 25



def validate_scene(scene: Scene, strict: bool = True) -> dict:
    """Validate scene geometry rules for RM65.

    Rules:
      1. 目标在工作空间内
      2. 目标不与任何障碍物重叠 (≥0.03m clearance)
      3. 目标不与机械臂基座太近 (XY ≥0.18m)
      4. 目标可达 (到基座距离 ≤ MAX_REACH)
      5. 障碍物不完全挡住基座
      6. 障碍到基座在合理范围内
      7. 目标不与 home 姿态手臂球重叠
    """
    goal = np.array(scene.goal_pos)
    issues = []

    # 1. Goal in workspace
    if not np.all((goal >= GOAL_MIN) & (goal <= GOAL_MAX)):
        issues.append(f"goal_outside_workspace({goal.tolist()})")

    # 2. Goal-obstacle clearance (skip is_table obstacles)
    for obs in scene.obstacles:
        if obs.is_table:
            continue
        c = np.array(obs.center)
        hs = np.array(obs.size) / 2
        # signed distance: negative = inside
        d = np.max(np.abs(goal - c) - hs)
        if d < 0.03:
            issues.append(f"goal_too_close_{obs.name}(d={d:.4f})")

    # 3. Goal-base XY clearance
    goal_xy_dist = np.sqrt(goal[0]**2 + goal[1]**2)
    if goal_xy_dist < 0.18:
        issues.append(f"goal_too_close_to_base(xy={goal_xy_dist:.3f})")

    # 4. Goal reachable
    goal_dist_base = np.sqrt(goal[0]**2 + goal[1]**2 + (goal[2] - 0.245)**2)
    if goal_dist_base > MAX_REACH:
        issues.append(f"goal_unreachable(d={goal_dist_base:.3f})")

    # 5. Base clearance from obstacles
    base_xy = np.array([0.0, 0.0])
    for obs in scene.obstacles:
        if obs.is_table:
            continue
        c_xy = np.array(obs.center[:2])
        hs_xy = np.array(obs.size[:2]) / 2
        if all(abs(base_xy[i] - c_xy[i]) < hs_xy[i] + 0.12 for i in range(2)):
            if obs.center[2] - obs.size[2]/2 < 0.30:
                issues.append(f"obs_blocks_base_{obs.name}")

    # 6. Obstacles in range
    for obs in scene.obstacles:
        if obs.is_table:
            continue
        c = np.array(obs.center)
        cdist = np.sqrt(c[0]**2 + c[1]**2 + (c[2] - 0.245)**2)
        if cdist > OBS_MAX_DIST:
            issues.append(f"obs_too_far_{obs.name}(d={cdist:.3f})")

    # 7. Goal clearance from home-pose arm spheres
    for (sx, sy, sz), sr in zip(ARM_SPHERES_HOME, ARM_SPHERE_RADII):
        d = np.sqrt((goal[0]-sx)**2 + (goal[1]-sy)**2 + (goal[2]-sz)**2)
        if d < sr + 0.02:
            issues.append(f"goal_overlaps_arm(d={d:.3f})")
            break

    return {
        "name": scene.name,
        "source": scene.source,
        "valid": len(issues) == 0,
        "issues": issues,
        "n_obstacles": len([o for o in scene.obstacles if not o.is_table]),
        "goal_pos": list(scene.goal_pos),
    }


def generate_variants(base_scene: Scene, n: int, seed: int = 42,
                      pos_jitter: float = 0.03, size_jitter: float = 0.12,
                      goal_jitter: float = 0.04, max_attempts: int = 500) -> list[Scene]:
    """Generate N validated variants by jittering obstacles and goal."""
    rng = np.random.RandomState(seed)
    variants = []
    attempts = 0

    while len(variants) < n and attempts < max_attempts:
        attempts += 1
        new_obs = []
        for obs in base_scene.obstacles:
            if obs.is_table:
                # 保持地面不变
                new_obs.append(Obstacle(
                    name=obs.name, center=obs.center, size=obs.size,
                    color=obs.color, alpha=obs.alpha, is_table=True,
                ))
                continue
            c = np.array(obs.center)
            s = np.array(obs.size)
            c_new = c + rng.uniform(-pos_jitter, pos_jitter, 3)
            s_new = s * (1.0 + rng.uniform(-size_jitter, size_jitter, 3))
            s_new = np.maximum(s_new, 0.015)
            new_obs.append(Obstacle(
                name=obs.name,
                center=tuple(c_new.tolist()),
                size=tuple(s_new.tolist()),
                color=obs.color,
                alpha=obs.alpha,
            ))

        g = np.array(base_scene.goal_pos)
        g_new = g + rng.uniform(-goal_jitter, goal_jitter, 3)
        g_new = np.clip(g_new, GOAL_MIN, GOAL_MAX)

        candidate = Scene(
            name=f"{base_scene.name}_v{len(variants)+1:03d}",
            goal_pos=tuple(g_new.tolist()),
            obstacles=new_obs,
            source=base_scene.source,
            category="variant",
            parent=base_scene.name,
            variant_id=len(variants) + 1,
        )
        vr = validate_scene(candidate, strict=True)
        if vr["valid"]:
            variants.append(candidate)

    return variants


def scene_to_dict(scene: Scene) -> dict:
    return {
        "name": scene.name,
        "source": scene.source,
        "category": scene.category,
        "parent": scene.parent,
        "variant_id": scene.variant_id,
        "goal_pos": list(scene.goal_pos),
        "obstacles": [
            {"name": o.name, "center": list(o.center), "size": list(o.size),
             "is_table": o.is_table}
            for o in scene.obstacles
        ],
    }
