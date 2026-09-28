"""Existing dynamic-obstacle proposal primitives (60 Hz).

These functions propose traces; downstream geometry and solvability gates
remain separate. The z=2 m parking state marks an inactive obstacle.
"""
from __future__ import annotations
from typing import Optional
import numpy as np
DT = 1.0 / 60.0

def _generate_goal_block_trace(
    *, goal_position, start, speed, total_steps, start_delay_steps,
    hold_steps, far_pos,
) -> np.ndarray:
    """球移到 goal 附近后长时间停留，模拟目标被阻挡。"""
    goal = np.asarray(goal_position, dtype=np.float64)
    # 球停留在 goal 正上方 5cm（不完全重叠但遮挡 approach）
    block_pos = goal.copy()
    block_pos[2] += 0.05

    distance = float(np.linalg.norm(block_pos - start))
    motion_steps = max(1, int(distance / max(speed * DT, 1e-6)))

    trace = np.zeros((total_steps, 1, 3), dtype=np.float64)
    for i in range(min(start_delay_steps, total_steps)):
        trace[i, 0, :] = far_pos
    for i in range(motion_steps + 1):
        step = start_delay_steps + i
        if step >= total_steps:
            break
        frac = float(i) / max(motion_steps, 1)
        trace[step, 0, :] = start + frac * (block_pos - start)
    # Extended hold at block position
    for i in range(hold_steps):
        step = start_delay_steps + motion_steps + i
        if step >= total_steps:
            break
        trace[step, 0, :] = block_pos
    for i in range(start_delay_steps + motion_steps + hold_steps, total_steps):
        trace[i, 0, :] = far_pos
    return trace


def _generate_head_on_trace(
    *, ee_trace, speed, total_steps, start_delay_steps, far_pos,
) -> np.ndarray:
    """球沿臂 EE 运动方向反向迎面而来。"""
    T_ee = len(ee_trace)
    mid_idx = T_ee // 2
    ee_mid = ee_trace[mid_idx].copy()
    # EE 在 mid 附近的运动方向
    look_back = max(0, mid_idx - 5)
    look_fwd = min(T_ee - 1, mid_idx + 5)
    ee_dir = ee_trace[look_fwd] - ee_trace[look_back]
    ee_dir_norm = np.linalg.norm(ee_dir)
    if ee_dir_norm < 1e-6:
        ee_dir = np.array([1.0, 0.0, 0.0])
    else:
        ee_dir = ee_dir / ee_dir_norm

    # 球从臂前方迎面而来
    approach_dist = 0.3
    start = ee_mid + ee_dir * approach_dist
    end = ee_mid - ee_dir * approach_dist

    distance = float(np.linalg.norm(end - start))
    motion_steps = max(1, int(distance / max(speed * DT, 1e-6)))

    trace = np.zeros((total_steps, 1, 3), dtype=np.float64)
    for i in range(min(start_delay_steps, total_steps)):
        trace[i, 0, :] = far_pos
    for i in range(motion_steps + 1):
        step = start_delay_steps + i
        if step >= total_steps:
            break
        frac = float(i) / max(motion_steps, 1)
        trace[step, 0, :] = start + frac * (end - start)
    for i in range(start_delay_steps + motion_steps, total_steps):
        trace[i, 0, :] = far_pos
    return trace


def _generate_near_miss_trace(
    *, start, end, speed, total_steps, start_delay_steps,
    hold_steps, far_pos, sphere_radius,
) -> np.ndarray:
    """球从侧面贴近臂轨迹但不穿越，擦边而过。

    在原始 start→end 方向上添加一个垂直偏移，使球从侧面经过。
    """
    direction = end - start
    dist = np.linalg.norm(direction)
    if dist < 1e-6:
        direction = np.array([0.0, 1.0, 0.0])
    else:
        direction = direction / dist

    # 找一个垂直于 direction 的偏移方向
    up = np.array([0.0, 0.0, 1.0])
    perp = np.cross(direction, up)
    if np.linalg.norm(perp) < 1e-6:
        perp = np.cross(direction, np.array([1.0, 0.0, 0.0]))
    perp = perp / np.linalg.norm(perp)

    # 偏移量：球半径 + 小余量，使球贴近但不碰撞
    offset = perp * (sphere_radius + 0.02)
    nm_start = start + offset
    nm_end = end + offset

    distance = float(np.linalg.norm(nm_end - nm_start))
    motion_steps = max(1, int(distance / max(speed * DT, 1e-6)))

    trace = np.zeros((total_steps, 1, 3), dtype=np.float64)
    for i in range(min(start_delay_steps, total_steps)):
        trace[i, 0, :] = far_pos
    for i in range(motion_steps + 1):
        step = start_delay_steps + i
        if step >= total_steps:
            break
        frac = float(i) / max(motion_steps, 1)
        trace[step, 0, :] = nm_start + frac * (nm_end - nm_start)
    for i in range(hold_steps):
        step = start_delay_steps + motion_steps + i
        if step >= total_steps:
            break
        trace[step, 0, :] = nm_end
    for i in range(start_delay_steps + motion_steps + hold_steps, total_steps):
        trace[i, 0, :] = far_pos
    return trace


def generate_sphere_trace(
    *,
    start: np.ndarray,
    end: np.ndarray,
    speed: float,
    total_steps: int,
    start_delay_steps: int,
    sphere_radius: float,
    hold_steps: int = 30,
    max_active_ratio: float | None = None,
    motion_mode: str = "linear",
    ee_trace: np.ndarray | None = None,
    goal_position: np.ndarray | None = None,
) -> np.ndarray:
    """生成球轨迹 (T, 1, 3).

    motion_mode:
      - "linear": 线性匀速 start→end（默认，向后兼容）
      - "goal_block": 球移到 goal 附近后长时间停留
      - "head_on": 球沿臂 EE 运动方向反向迎面而来
      - "near_miss": 球从侧面贴近臂轨迹但不穿越，擦边而过
    """
    far_pos = np.array([0.0, 0.0, 2.0])

    if motion_mode == "goal_block" and goal_position is not None:
        return _generate_goal_block_trace(
            goal_position=goal_position, start=start, speed=speed,
            total_steps=total_steps, start_delay_steps=start_delay_steps,
            hold_steps=hold_steps * 3, far_pos=far_pos,
        )

    if motion_mode == "head_on" and ee_trace is not None:
        return _generate_head_on_trace(
            ee_trace=ee_trace, speed=speed, total_steps=total_steps,
            start_delay_steps=start_delay_steps, far_pos=far_pos,
        )

    if motion_mode == "near_miss":
        return _generate_near_miss_trace(
            start=start, end=end, speed=speed, total_steps=total_steps,
            start_delay_steps=start_delay_steps, hold_steps=hold_steps,
            far_pos=far_pos, sphere_radius=sphere_radius,
        )

    # ── Default: linear mode (original logic) ──
    distance = float(np.linalg.norm(end - start))
    motion_steps = max(1, int(distance / max(speed * DT, 1e-6)))
    if max_active_ratio is not None:
        active_budget = max(1, int(total_steps * max_active_ratio))
        if motion_steps + hold_steps > active_budget:
            hold_steps = max(0, min(hold_steps, active_budget - min(motion_steps, active_budget)))
            if motion_steps + hold_steps > active_budget:
                motion_steps = max(1, active_budget - hold_steps)
    direction = (end - start) / max(distance, 1e-9)

    trace = np.zeros((total_steps, 1, 3), dtype=np.float64)
    # Before start: hold at start (well above workspace, invisible)
    far_pos = np.array([0.0, 0.0, 2.0])  # far away
    for i in range(min(start_delay_steps, total_steps)):
        trace[i, 0, :] = far_pos

    # Motion phase: linear interpolation
    for i in range(motion_steps + 1):
        step = start_delay_steps + i
        if step >= total_steps:
            break
        frac = float(i) / max(motion_steps, 1)
        trace[step, 0, :] = start + frac * (end - start)

    # After motion: hold at end, then move to far away
    hold_steps = int(hold_steps)
    for i in range(hold_steps):
        step = start_delay_steps + motion_steps + i
        if step >= total_steps:
            break
        trace[step, 0, :] = end

    # Remaining steps: far away
    for i in range(start_delay_steps + motion_steps + hold_steps, total_steps):
        trace[i, 0, :] = far_pos

    return trace


def _axis_span_endpoints(center: np.ndarray, axis: str, span: float) -> tuple[np.ndarray, np.ndarray]:
    """Compute start/end for a span along *axis* centered on *center*.

    Supports: x, y, z, xz, yz, xy.  "perp_xy*" variants are normalised to "x".
    """
    norm = axis.lower().replace("perp_xy", "x")
    for sfx in ("_p15", "_m15", "_p30", "_m30"):
        norm = norm.replace(sfx, "")
    _DIRS = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
        "xz": np.array([1.0, 0.0, 1.0]),
        "yz": np.array([0.0, 1.0, 1.0]),
        "xy": np.array([1.0, 1.0, 0.0]),
    }
    d = _DIRS.get(norm, _DIRS["y"])
    d = d / np.linalg.norm(d)
    start = center + d * (span / 2)
    end = center - d * (span / 2)
    return start, end


def compute_smart_endpoints(
    ee_trace: np.ndarray,
    template_window: dict,
    total_steps: int,
    event_window: dict | None = None,
) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """从臂 EE 轨迹和模板窗口计算确保碰撞的球起终点.

    如果提供了 sidecar event_window, 则使用窗口中间点的 EE 位置 (更精准);
    否则回退到整体 T//2 中间点.
    """
    axis_str = template_window.get("axis", "y")
    span = float(template_window.get("path_span", 0.30))

    # Map the target phase into ee-trace index space.
    if event_window is not None:
        mid_step = 0.5 * (float(event_window["start"]) + float(event_window["end"]))
        phase = mid_step / max(float(total_steps - 1), 1.0)
    else:
        phase = float(template_window.get("path_fraction", 0.30))
    phase = float(np.clip(phase, 0.0, 1.0))
    mid_idx = int(round(phase * max(len(ee_trace) - 1, 0)))
    center_on_path = ee_trace[mid_idx].copy()
    offset_vec = np.array(
        [
            float(template_window.get("offset_x", 0.0)),
            float(template_window.get("offset_y", 0.0)),
            float(template_window.get("offset_z", 0.0)),
        ],
        dtype=np.float64,
    )

    def _score_candidate(start_: np.ndarray, end_: np.ndarray) -> float:
        mid = 0.5 * (start_ + end_)
        min_mid_dist = float(np.min(np.linalg.norm(ee_trace - mid[None, :], axis=1)))
        start_dist0 = float(np.linalg.norm(start_ - ee_trace[0]))
        safety_penalty = max(0.0, 0.15 - start_dist0) * 4.0
        return min_mid_dist + safety_penalty

    # If the template already encodes a legal crossing segment, preserve its
    # relative geometry and just translate it onto the current nominal path.
    if "path_start" in template_window and "path_end" in template_window:
        t_start = np.asarray(template_window["path_start"], dtype=np.float64)
        t_end = np.asarray(template_window["path_end"], dtype=np.float64)
        t_mid = 0.5 * (t_start + t_end)
        centered_start = center_on_path + (t_start - t_mid)
        centered_end = center_on_path + (t_end - t_mid)
        offset_start = center_on_path + offset_vec + (t_start - t_mid)
        offset_end = center_on_path + offset_vec + (t_end - t_mid)
        start, end = min(
            [(centered_start, centered_end), (offset_start, offset_end)],
            key=lambda pair: _score_candidate(pair[0], pair[1]),
        )
    else:
        # Fallback: cross the path around the target center.
        center = center_on_path + offset_vec
        start, end = _axis_span_endpoints(center, axis_str, span)

    return start, end
