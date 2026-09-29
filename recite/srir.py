"""Route-static SR-IR kernels from the existing compiler.

The historical function name find_event_windows is retained for source
compatibility. These intervals are Interaction-candidate intervals (ICI),
with [start, end) route-index semantics.
"""
from __future__ import annotations
import numpy as np

def find_event_windows(
    clearance_profile: np.ndarray,
    threshold: float = 0.08,
    min_window_steps: int = 10,
) -> list[dict]:
    """从 clearance profile 提取事件窗口 (臂接近障碍物的时段).

    事件窗口 = clearance < threshold 的连续帧段.
    """
    in_window = clearance_profile < threshold
    windows = []
    start = None

    for t in range(len(in_window)):
        if in_window[t] and start is None:
            start = t
        elif not in_window[t] and start is not None:
            if t - start >= min_window_steps:
                windows.append({
                    "start": int(start),
                    "end": int(t),
                    "duration": int(t - start),
                    "min_clearance": float(clearance_profile[start:t].min()),
                    "reason": "near_obstacle",
                })
            start = None

    # Handle window at end
    if start is not None and len(in_window) - start >= min_window_steps:
        windows.append({
            "start": int(start),
            "end": int(len(in_window)),
            "duration": int(len(in_window) - start),
            "min_clearance": float(clearance_profile[start:].min()),
            "reason": "near_obstacle",
        })

    return windows


def _find_continuous_segments(mask: np.ndarray, min_steps: int) -> list[tuple[int, int]]:
    """从 bool mask 中提取连续 True 段 (≥min_steps)."""
    segments = []
    start = None
    T = len(mask)
    for t in range(T):
        if mask[t] and start is None:
            start = t
        elif not mask[t] and start is not None:
            if t - start >= min_steps:
                segments.append((start, t))
            start = None
    if start is not None and T - start >= min_steps:
        segments.append((start, T))
    return segments


def build_clearance_profile(clearance: np.ndarray) -> dict:
    """Serialize the full route-static clearance signal and its summary.

    Summary statistics preserve the sidecar interface, while ``values_m``
    retains the per-step signal for reproducible event-window and ablation
    decisions.
    """
    values = np.asarray(clearance, dtype=np.float64).reshape(-1)
    if (
        values.size == 0
        or np.any(np.isnan(values))
        or np.any(np.isneginf(values))
    ):
        raise ValueError(
            "clearance profile must contain finite or positive-unbounded samples"
        )
    finite = values[np.isfinite(values)]
    unbounded_count = int(np.count_nonzero(np.isposinf(values)))
    return {
        "schema": "recite_clearance_profile_v1",
        "sample_count": int(values.size),
        "values_m": [
            float(value) if np.isfinite(value) else None for value in values
        ],
        "unbounded_step_count": unbounded_count,
        "all_steps_unbounded": bool(unbounded_count == values.size),
        "finite_summary_scope": "finite_samples_only",
        "min": float(finite.min()) if finite.size else None,
        "max": (
            None
            if unbounded_count
            else float(finite.max()) if finite.size else None
        ),
        "mean": float(finite.mean()) if finite.size else None,
        "std": float(finite.std()) if finite.size else None,
        "steps_below_005": int(np.count_nonzero(values < 0.05)),
        "steps_below_008": int(np.count_nonzero(values < 0.08)),
    }


def build_occupied_sphere_tube(
    sphere_positions: dict[str, np.ndarray],
    robot_spec: dict,
) -> dict:
    """Build the reconstructable swept-volume contract for one nominal route.

    Each collision sphere has constant radius and a linearly interpolated
    centre between consecutive route samples.  Its continuous sweep over one
    segment is therefore a capsule.  The union of these capsules is the
    occupied tube under this explicit interpolation contract.
    """
    sphere_ids = [str(value) for value in robot_spec["collision_sphere_ids"]]
    if not sphere_ids or set(sphere_positions) != set(sphere_ids):
        raise ValueError("occupied tube sphere identities disagree with geometry")
    radius_by_id = {
        str(row["sphere_id"]): float(row["radius_m"])
        for row in robot_spec["collision_spheres"]
    }
    arrays = []
    sample_count = None
    for sphere_id in sphere_ids:
        value = np.asarray(sphere_positions[sphere_id], dtype=np.float64)
        if value.ndim != 2 or value.shape[1] != 3 or not np.all(np.isfinite(value)):
            raise ValueError(f"invalid occupied tube centres for {sphere_id}")
        if sample_count is None:
            sample_count = len(value)
        elif len(value) != sample_count:
            raise ValueError("occupied tube sphere traces are not time aligned")
        arrays.append(value)
    if not sample_count:
        raise ValueError("occupied tube requires at least one route sample")
    centres = np.stack(arrays, axis=1)
    radii = np.asarray([radius_by_id[sphere_id] for sphere_id in sphere_ids])
    occupied_min = np.min(centres - radii[None, :, None], axis=(0, 1))
    occupied_max = np.max(centres + radii[None, :, None], axis=(0, 1))
    centroid = centres.mean(axis=1)
    return {
        "schema": "recite_time_indexed_swept_spheres_v1",
        "geometry_id": str(robot_spec["geometry_id"]),
        "sample_count": int(sample_count),
        "segment_count": max(int(sample_count) - 1, 0),
        "sphere_count": len(sphere_ids),
        "sphere_ids": sphere_ids,
        "sphere_radii_m": [float(value) for value in radii],
        "centers_m": centres.tolist(),
        "center_interpolation": "piecewise_linear_in_route_index",
        "continuous_time_semantics": (
            "linear_center_sweep_with_constant_radius_capsules"
        ),
        "union_semantics": "union_over_spheres_and_route_segments",
        "bbox_min": occupied_min.tolist(),
        "bbox_max": occupied_max.tolist(),
        "max_extent": float(np.linalg.norm(occupied_max - occupied_min)),
        "centroid_trajectory_shape": list(centroid.shape),
    }


def _merge_portal_segments(
    clearance_segments: list[tuple[int, int]],
    reachability_segments: list[tuple[int, int]],
    clearance: np.ndarray,
    reachability_profile: np.ndarray | None,
    merge_gap: int = 5,
) -> list[tuple[int, int, str]]:
    """合并 clearance + reachability portal 段 → (start, end, source).

    重叠段合并 → source="both"; 间距 ≤ merge_gap 的连续段也合并.
    """
    # 标注 source
    tagged: list[tuple[int, int, str]] = []
    for s, e in clearance_segments:
        tagged.append((s, e, "clearance"))
    for s, e in reachability_segments:
        tagged.append((s, e, "reachability"))

    if not tagged:
        return []

    # 按 start 排序
    tagged.sort(key=lambda x: x[0])

    # 合并重叠/相邻段
    merged: list[tuple[int, int, str]] = [tagged[0]]
    for s, e, src in tagged[1:]:
        prev_s, prev_e, prev_src = merged[-1]
        if s <= prev_e + merge_gap:
            # 重叠或紧邻 → 合并
            new_src = "both" if prev_src != src else prev_src
            if prev_src == "both" or src == "both":
                new_src = "both"
            merged[-1] = (prev_s, max(prev_e, e), new_src)
        else:
            merged.append((s, e, src))

    return merged


def extract_portal_graph(
    clearance: np.ndarray,
    ee_traj: np.ndarray,
    portal_threshold: float = 0.12,
    region_threshold: float = 0.10,
    min_portal_steps: int = 8,
    reachability_profile: np.ndarray | None = None,
    reachability_threshold: float = 0.10,
    min_reachability_steps: int = 5,
) -> dict:
    """从 clearance profile + reachability profile 提取 dual-channel portal graph.

    Clearance portal: clearance < portal_threshold (碰撞风险瓶颈)
    Reachability portal: density < reachability_threshold (运动学瓶颈)
    两者互补: 有些区域 clearance 还行但姿态受限; 有些区域很窄但仍有多种姿态.

    Returns:
        {
            "nodes": [{"id": str, "type": "region"|"portal", "portal_source": str, ...}],
            "edges": [...],
            "n_portals": int,
            "n_clearance_portals": int,
            "n_reachability_portals": int,
            "n_both_portals": int,
            "min_portal_clearance": float,
        }
    """
    T = len(clearance)

    # Clearance portals
    clearance_segs = _find_continuous_segments(clearance < portal_threshold, min_portal_steps)

    # Reachability portals (if available)
    reach_segs = []
    if reachability_profile is not None:
        reach_segs = _find_continuous_segments(
            reachability_profile < reachability_threshold,
            min_reachability_steps,
        )

    # 合并两个通道
    portals_merged = _merge_portal_segments(
        clearance_segs, reach_segs, clearance, reachability_profile,
    )

    # === 以下逻辑与 v1 相同，但 portal 带 source 标签 ===
    portals = portals_merged  # list of (start, end, source)

    # 构建 node/edge graph
    nodes = []
    edges = []

    # 起始区域
    if portals:
        first_portal_start = portals[0][0]
    else:
        first_portal_start = T

    nodes.append({
        "id": "home_region",
        "type": "region",
        "step_range": [0, first_portal_start],
        "mean_clearance": float(clearance[:max(1, first_portal_start)].mean()),
    })

    for pi, (ps, pe, source) in enumerate(portals):
        # Portal node
        portal_id = f"portal_{pi}"
        mc = float(clearance[ps:pe].min())
        critical_step = int(ps + np.argmin(clearance[ps:pe]))
        portal_pos = (
            ee_traj[critical_step].tolist()
            if ee_traj is not None and critical_step < len(ee_traj)
            else None
        )

        route_tangent = None
        lateral_normal = None
        route_span = None
        if ee_traj is not None and len(ee_traj) > 0:
            entry_step = min(max(int(ps), 0), len(ee_traj) - 1)
            exit_step = min(max(int(pe) - 1, entry_step), len(ee_traj) - 1)
            local_step = min(max(critical_step, entry_step), exit_step)
            before = max(0, local_step - 1)
            after = min(len(ee_traj) - 1, local_step + 1)
            tangent = np.asarray(
                ee_traj[after] - ee_traj[before],
                dtype=np.float64,
            )
            tangent_norm = float(np.linalg.norm(tangent))
            portal_route = np.asarray(
                ee_traj[entry_step : exit_step + 1], dtype=np.float64
            )
            route_span = float(
                np.sum(np.linalg.norm(np.diff(portal_route, axis=0), axis=1))
            ) if len(portal_route) > 1 else 0.0
            if tangent_norm <= 1e-9:
                before = max(0, entry_step - 1)
                after = min(len(ee_traj) - 1, exit_step + 1)
                tangent = np.asarray(
                    ee_traj[after] - ee_traj[before],
                    dtype=np.float64,
                )
                tangent_norm = float(np.linalg.norm(tangent))
            if tangent_norm > 1e-9:
                tangent /= tangent_norm
                normal = np.asarray([-tangent[1], tangent[0], 0.0])
                normal_norm = float(np.linalg.norm(normal))
                if normal_norm <= 1e-9:
                    normal = np.cross(tangent, np.asarray([0.0, 1.0, 0.0]))
                    normal_norm = float(np.linalg.norm(normal))
                if normal_norm > 1e-9:
                    normal /= normal_norm
                    lateral_normal = normal.tolist()
                route_tangent = tangent.tolist()

        # Reachability density stats (if available)
        reach_stats = {}
        if reachability_profile is not None:
            reach_stats = {
                "density_min": float(reachability_profile[ps:pe].min()),
                "density_mean": float(reachability_profile[ps:pe].mean()),
            }

        nodes.append({
            "id": portal_id,
            "type": "portal",
            "portal_source": source,
            "step_range": [int(ps), int(pe)],
            "critical_step": critical_step,
            "min_clearance": mc,
            "position": portal_pos,
            "route_tangent_m": route_tangent,
            "lateral_normal_m": lateral_normal,
            "route_tangent_unit": route_tangent,
            "route_lateral_direction_unit": lateral_normal,
            "direction_vector_units": "unitless",
            "route_span_m": route_span,
            **reach_stats,
        })

        # Edge from previous region
        prev_region = nodes[-2]["id"] if len(nodes) >= 2 else "home_region"
        edges.append({"src": prev_region, "dst": portal_id, "steps": int(ps - nodes[-2].get("step_range", [0, ps])[1])})

        # Region after portal (unless last)
        if pi < len(portals) - 1:
            next_portal_start = portals[pi + 1][0]
        else:
            next_portal_start = T

        region_id = f"region_{pi + 1}" if pi < len(portals) - 1 else "goal_region"
        region_clearance = clearance[pe:max(pe + 1, next_portal_start)]
        nodes.append({
            "id": region_id,
            "type": "region",
            "step_range": [int(pe), int(next_portal_start)],
            "mean_clearance": float(region_clearance.mean()) if len(region_clearance) > 0 else 0.0,
        })
        edges.append({"src": portal_id, "dst": region_id, "steps": int(next_portal_start - pe)})

    if not portals:
        # No portals — single open region
        nodes.append({
            "id": "goal_region",
            "type": "region",
            "step_range": [0, T],
            "mean_clearance": float(clearance.mean()),
        })
        edges.append({"src": "home_region", "dst": "goal_region", "steps": T})

    min_pc = float(min(n["min_clearance"] for n in nodes if n["type"] == "portal")) if portals else float("inf")

    # 统计各来源 portal 数量
    n_clear = sum(1 for n in nodes if n["type"] == "portal" and n.get("portal_source") == "clearance")
    n_reach = sum(1 for n in nodes if n["type"] == "portal" and n.get("portal_source") == "reachability")
    n_both  = sum(1 for n in nodes if n["type"] == "portal" and n.get("portal_source") == "both")

    return {
        "schema": "recite_portal_graph_v1",
        "nodes": nodes,
        "edges": edges,
        "n_portals": len(portals),
        "n_clearance_portals": n_clear,
        "n_reachability_portals": n_reach,
        "n_both_portals": n_both,
        "min_portal_clearance": min_pc,
    }


def compute_rejoin_anchors(
    clearance: np.ndarray,
    ee_traj: np.ndarray,
    event_windows: list[dict] | None = None,
    min_clearance_for_rejoin: float = 0.08,
    min_step_gap: int = 30,
) -> list[dict]:
    """Return the first route-static safe rejoin after each event window.

    Each selected point and its short forward nominal segment satisfy the
    static-clearance threshold. The controller's online geometry check
    evaluates the bridge from its current off-route state to the anchor.
    """
    values = np.asarray(clearance, dtype=np.float64).reshape(-1)
    route = np.asarray(ee_traj, dtype=np.float64)
    if (
        values.size == 0
        or np.any(np.isnan(values))
        or np.any(np.isneginf(values))
    ):
        raise ValueError(
            "rejoin anchors require finite or positive-unbounded clearances"
        )
    if route.shape != (len(values), 3) or not np.all(np.isfinite(route)):
        raise ValueError("rejoin anchors require a time-aligned EE route")
    if min_step_gap < 1:
        raise ValueError("min_step_gap must be positive")
    windows = list(event_windows or [])
    if not windows:
        return []

    anchors: list[dict] = []
    used_steps: set[int] = set()
    previous_step = -int(min_step_gap)
    for event_index, window in enumerate(windows):
        start = int(window.get("start", -1))
        end = int(window.get("end", -1))
        if start < 0 or end <= start or end > len(values):
            raise ValueError(f"invalid event window for rejoin: {window}")
        search_start = max(end, previous_step + int(min_step_gap))
        selected = None
        for step in range(search_start, len(values)):
            confirmation_stop = min(
                len(values),
                step + max(1, min(3, int(min_step_gap))),
            )
            segment = values[step:confirmation_stop]
            if len(segment) and float(segment.min()) >= min_clearance_for_rejoin:
                selected = (step, float(segment.min()), confirmation_stop)
                break
        if selected is None:
            continue
        step, segment_min, confirmation_stop = selected
        if step in used_steps:
            continue
        used_steps.add(step)
        previous_step = step
        anchors.append({
            "step": int(step),
            "clearance": float(values[step]),
            "position": route[step].tolist(),
            "event_window_index": int(event_index),
            "valid_after_step": int(end),
            "segment_end_exclusive": int(confirmation_stop),
            "segment_min_clearance_m": segment_min,
            "minimum_required_clearance_m": float(min_clearance_for_rejoin),
            "certificate": "nominal_route_static_clearance",
        })
    return anchors


def compute_goal_approach_cone(ee_traj: np.ndarray, goal: np.ndarray, n_final_steps: int = 30) -> dict:
    """计算接近目标的方向锥.

    从最后 n_final_steps 的 EE 运动方向拟合一个锥体.
    """
    goal = np.asarray(goal, dtype=np.float64).reshape(-1)
    if goal.shape != (3,) or not np.all(np.isfinite(goal)):
        raise ValueError("goal approach cone requires a finite 3D goal")

    def invalid(reason: str) -> dict:
        return {
            "schema": "recite_goal_approach_cone_v1",
            "valid": False,
            "invalid_reason": reason,
            "apex_m": goal.tolist(),
            "axis": [0.0, 0.0, 0.0],
            "half_angle_deg": 90.0,
            "source_step_range": None,
        }

    if ee_traj is None or len(ee_traj) < n_final_steps:
        return invalid("insufficient_route_samples")

    final_segment = ee_traj[-n_final_steps:]
    approach_vectors = np.diff(final_segment, axis=0)
    norms = np.linalg.norm(approach_vectors, axis=1, keepdims=True)
    valid = norms.flatten() > 1e-6
    if not np.any(valid):
        return invalid("stationary_final_segment")

    unit_vectors = approach_vectors[valid] / norms[valid]
    mean_axis = unit_vectors.mean(axis=0)
    mean_norm = np.linalg.norm(mean_axis)
    if mean_norm < 1e-6:
        return invalid("inconsistent_approach_directions")

    mean_axis /= mean_norm

    # 计算半角
    cos_angles = np.clip(np.dot(unit_vectors, mean_axis), -1, 1)
    half_angle = float(np.degrees(np.arccos(cos_angles.min())))

    return {
        "schema": "recite_goal_approach_cone_v1",
        "valid": True,
        "invalid_reason": None,
        "apex_m": goal.tolist(),
        "axis": mean_axis.tolist(),
        "half_angle_deg": round(half_angle, 1),
        "source_step_range": [
            int(len(ee_traj) - n_final_steps),
            int(len(ee_traj)),
        ],
    }
