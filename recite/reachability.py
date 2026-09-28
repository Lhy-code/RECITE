"""Workspace-density queries and legacy aggregate reachability score.

Density profiles and the historical weighted aggregate are separate outputs;
compute_true_reachability_score is not the mean route-density statistic.
"""
from __future__ import annotations
import numpy as np

def build_density_histogram(
    free_ee_pos: np.ndarray,
    resolution: float = 0.03,
    workspace_bounds: tuple[tuple[float, ...], ...] | None = None,
) -> tuple[np.ndarray, list[np.ndarray]]:
    """构建 3D workspace 密度直方图 H[x,y,z].

    Args:
        free_ee_pos: (M, 3) 无碰撞 EE 位置
        resolution: 体素分辨率 (m)
        workspace_bounds: ((x_min,x_max),(y_min,y_max),(z_min,z_max))
                         默认从数据推断 + margin

    Returns:
        H: 3D histogram (counts)
        edges: bin edges list [x_edges, y_edges, z_edges]
    """
    if workspace_bounds is None:
        # 从数据推断 + 0.05m margin
        margin = 0.05
        mins = free_ee_pos.min(axis=0) - margin
        maxs = free_ee_pos.max(axis=0) + margin
        workspace_bounds = tuple((float(lo), float(hi)) for lo, hi in zip(mins, maxs))

    bins = []
    for lo, hi in workspace_bounds:
        n_bins = max(2, int(np.ceil((hi - lo) / resolution)))
        bins.append(np.linspace(lo, hi, n_bins + 1))

    H, edges = np.histogramdd(free_ee_pos, bins=bins)
    return H, edges


def query_density_along_trajectory(
    ee_trajectory: np.ndarray,
    H: np.ndarray,
    edges: list[np.ndarray],
    normalize: bool = True,
) -> np.ndarray:
    """沿 EE 轨迹查询 density → reachability profile.

    Args:
        ee_trajectory: (T, 3) 参考 EE 轨迹
        H: 3D histogram
        edges: bin edges
        normalize: 是否归一化到 [0, 1]

    Returns:
        profile: (T,) 每步的 density 值
    """
    T = len(ee_trajectory)
    profile = np.zeros(T, dtype=np.float32)

    for t in range(T):
        pt = ee_trajectory[t]
        # 找到所属 voxel
        ix = np.searchsorted(edges[0], pt[0]) - 1
        iy = np.searchsorted(edges[1], pt[1]) - 1
        iz = np.searchsorted(edges[2], pt[2]) - 1

        if (0 <= ix < H.shape[0] and
            0 <= iy < H.shape[1] and
            0 <= iz < H.shape[2]):
            profile[t] = H[ix, iy, iz]

    if normalize and profile.max() > 0:
        profile = profile / H.max()

    return profile


def extract_reachability_portals(
    reachability_profile: np.ndarray,
    ee_trajectory: np.ndarray | None = None,
    density_threshold: float = 0.10,
    min_portal_steps: int = 5,
) -> list[dict]:
    """从 reachability profile 中提取 "reachability portal" 段.

    与 clearance portal 逻辑类似，但指标不同:
      闲 < density_threshold 的连续段 = reachability 瓶颈

    Args:
        reachability_profile: (T,) normalized [0,1]
        ee_trajectory: (T,3) 可选，用于记录 portal 位置
        density_threshold: 低于此密度视为 portal
        min_portal_steps: 最短持续步数

    Returns:
        list of portal dicts
    """
    T = len(reachability_profile)
    is_portal = reachability_profile < density_threshold

    portals = []
    start = None
    for t in range(T):
        if is_portal[t] and start is None:
            start = t
        elif not is_portal[t] and start is not None:
            if t - start >= min_portal_steps:
                mid = (start + t) // 2
                pos = ee_trajectory[mid].tolist() if ee_trajectory is not None and mid < len(ee_trajectory) else None
                portals.append({
                    "step_range": [int(start), int(t)],
                    "density_min": float(reachability_profile[start:t].min()),
                    "density_mean": float(reachability_profile[start:t].mean()),
                    "position": pos,
                    "type": "reachability",
                })
            start = None
    if start is not None and T - start >= min_portal_steps:
        mid = (start + T) // 2
        pos = ee_trajectory[mid].tolist() if ee_trajectory is not None and mid < len(ee_trajectory) else None
        portals.append({
            "step_range": [int(start), int(T)],
            "density_min": float(reachability_profile[start:T].min()),
            "density_mean": float(reachability_profile[start:T].mean()),
            "position": pos,
            "type": "reachability",
        })

    return portals


def compute_true_reachability_score(
    reachability_profile: np.ndarray,
    free_ratio: float,
) -> float:
    """计算真正的可达性评分 — 替代 sidecar_v2 的 reachability_proxy.

    综合指标:
      - mean_density: 轨迹沿线平均密度 (高 = 多数步可达性好)
      - min_density: 最低密度 (低 = 存在瓶颈)
      - free_ratio: 全空间无碰撞比例 (高 = 空间开阔)
      - consistency: 密度标准差的反数 (高 = 密度均匀)

    Returns:
        score in [0, 1]
    """
    if len(reachability_profile) == 0:
        return 0.0

    mean_d = float(reachability_profile.mean())
    min_d = float(reachability_profile.min())
    std_d = float(reachability_profile.std())
    consistency = float(np.clip(1.0 - std_d / max(mean_d, 0.01), 0, 1))

    score = (
        0.35 * mean_d +
        0.25 * min_d +
        0.20 * min(free_ratio * 2.5, 1.0) +   # free_ratio ~0.4 → 1.0
        0.20 * consistency
    )
    return float(np.clip(score, 0, 1))
