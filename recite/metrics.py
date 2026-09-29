"""Existing sampled geometry and first-contact severity kernels.

Inputs are robot-sphere centers and radii. Contact diagnostics measure
geometric overlap in metres and relative normal approach speed in metres/second.
"""
from __future__ import annotations
from typing import Any
import numpy as np

def _signed_point_aabb_distance(
    point: np.ndarray, center: np.ndarray, size: np.ndarray
) -> float:
    delta = np.abs(point - center) - size / 2.0
    outside = float(np.linalg.norm(np.maximum(delta, 0.0)))
    inside = min(float(np.max(delta)), 0.0)
    return outside + inside


def compute_dynamic_contact_severity(
    *,
    robot_centers: np.ndarray,
    raw_robot_radii: np.ndarray,
    robot_links: np.ndarray,
    dynamic_centers: np.ndarray,
    dynamic_radii: np.ndarray,
    dynamic_active: np.ndarray,
    time_s: np.ndarray,
) -> dict[str, Any]:
    """Compute sampled zero-buffer overlap and first-contact approach speed."""
    robot_centers = np.asarray(robot_centers, dtype=np.float64)
    raw_robot_radii = np.asarray(raw_robot_radii, dtype=np.float64)
    robot_links = np.asarray(robot_links)
    dynamic_centers = np.asarray(dynamic_centers, dtype=np.float64)
    dynamic_radii = np.asarray(dynamic_radii, dtype=np.float64).reshape(-1)
    dynamic_active = np.asarray(dynamic_active, dtype=bool)
    time_s = np.asarray(time_s, dtype=np.float64).reshape(-1)
    if robot_centers.ndim != 3 or robot_centers.shape[2] != 3:
        raise ValueError("severity robot centers must have shape (T, S, 3)")
    frames, spheres, _ = robot_centers.shape
    if raw_robot_radii.shape != (frames, spheres) or robot_links.shape != (spheres,):
        raise ValueError("severity robot radii/links do not match centers")
    if dynamic_centers.ndim != 3 or dynamic_centers.shape[0] != frames:
        raise ValueError("severity dynamic centers do not match the time axis")
    dynamic_count = dynamic_centers.shape[1]
    if (
        dynamic_centers.shape[2] != 3
        or dynamic_radii.shape != (dynamic_count,)
        or dynamic_active.shape != (frames, dynamic_count)
        or time_s.shape != (frames,)
    ):
        raise ValueError("severity dynamic geometry/time shapes are inconsistent")
    if not np.all(np.diff(time_s) > 0.0):
        raise ValueError("severity time axis must be strictly increasing")
    pairwise = (
        np.linalg.norm(
            robot_centers[:, :, None, :] - dynamic_centers[:, None, :, :],
            axis=-1,
        )
        - raw_robot_radii[:, :, None]
        - dynamic_radii[None, None, :]
    )
    pairwise = np.where(dynamic_active[:, None, :], pairwise, np.inf)
    minimum = np.min(pairwise, axis=(1, 2))
    contact_frames = np.flatnonzero(minimum <= 0.0)
    maximum_overlap_m = float(max(0.0, -np.min(minimum)))
    if not len(contact_frames):
        return {
            "maximum_zero_buffer_dynamic_overlap_m": maximum_overlap_m,
            "first_dynamic_contact_frame": None,
            "first_dynamic_contact_link": None,
            "first_dynamic_contact_normal_approach_speed_m_s": None,
            "approach_speed_semantics": (
                "sampled_relative_normal_speed_at_first_zero_buffer_contact"
            ),
        }
    frame = int(contact_frames[0])
    robot_index, dynamic_index = np.unravel_index(
        int(np.argmin(pairwise[frame])), pairwise[frame].shape
    )
    approach_speed: float | None = None
    if frame > 0 and bool(dynamic_active[frame - 1, dynamic_index]):
        dt = float(time_s[frame] - time_s[frame - 1])
        separation = (
            robot_centers[frame, robot_index]
            - dynamic_centers[frame, dynamic_index]
        )
        norm = float(np.linalg.norm(separation))
        if norm > 0.0:
            normal = separation / norm
            robot_velocity = (
                robot_centers[frame, robot_index]
                - robot_centers[frame - 1, robot_index]
            ) / dt
            obstacle_velocity = (
                dynamic_centers[frame, dynamic_index]
                - dynamic_centers[frame - 1, dynamic_index]
            ) / dt
            approach_speed = float(
                max(0.0, -np.dot(normal, robot_velocity - obstacle_velocity))
            )
    return {
        "maximum_zero_buffer_dynamic_overlap_m": maximum_overlap_m,
        "first_dynamic_contact_frame": frame,
        "first_dynamic_contact_link": str(robot_links[robot_index]),
        "first_dynamic_contact_normal_approach_speed_m_s": approach_speed,
        "approach_speed_semantics": (
            "sampled_relative_normal_speed_at_first_zero_buffer_contact"
        ),
    }
