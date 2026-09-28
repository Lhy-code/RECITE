#!/usr/bin/env python3
"""Run released CPU stages on a recorded reference route, without simulation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from recite.dynamic import DT, compute_smart_endpoints, generate_sphere_trace
from recite.metrics import _signed_point_aabb_distance, compute_dynamic_contact_severity
from recite.scenes import ALL_BASE_SCENES, generate_variants, scene_to_dict, validate_scene
from recite.srir import (
    build_clearance_profile,
    build_occupied_sphere_tube,
    compute_goal_approach_cone,
    compute_rejoin_anchors,
    extract_portal_graph,
    find_event_windows,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "data/rm65_open_field_v048.json")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/sample")
    args = parser.parse_args()
    sample = json.loads(args.input.read_text(encoding="utf-8"))
    args.output.mkdir(parents=True, exist_ok=True)

    # Static sampling is a separate stage demonstration. These newly sampled
    # variants are NOT paired with the archived route below.
    scene_results = [validate_scene(scene) for scene in ALL_BASE_SCENES.values()]
    variants = generate_variants(ALL_BASE_SCENES["rm65_corridor"], n=3, seed=42)
    (args.output / "static_variants.json").write_text(
        json.dumps([scene_to_dict(v) for v in variants], indent=2), encoding="utf-8"
    )

    # Reuse an already planned route and its recorded FK centers. No planner,
    # robot asset, or GPU is required to recompile these numerical fields.
    ee = np.asarray(sample["ee"], dtype=float)
    centers = np.asarray(sample["sphere_centers_m"], dtype=float)
    geometry = sample["geometry"]
    radii = np.asarray([row["radius_m"] for row in geometry["collision_spheres"]])
    obstacles = np.asarray(sample["obs_centers"], dtype=float)
    sizes = np.asarray(sample["obs_sizes"], dtype=float)
    dt = float(sample["timebase"]["dt_s"])
    if not np.isclose(dt, DT):
        raise ValueError("The released proposal primitives use a 60 Hz timebase.")
    clearance = np.full(len(ee), np.inf)
    for t, sphere_centers in enumerate(centers):
        for center, radius in zip(sphere_centers, radii):
            for obstacle, size in zip(obstacles, sizes):
                clearance[t] = min(
                    clearance[t], _signed_point_aabb_distance(center, obstacle, size) - radius
                )
    np.testing.assert_allclose(clearance, sample["recorded_clearance_m"], atol=1e-10)
    ici = find_event_windows(clearance)
    srir = {
        "scene_id": sample["scene_id"],
        "timebase": sample["timebase"],
        "clearance_profile": build_clearance_profile(clearance),
        "occupied_tube": build_occupied_sphere_tube(
            {name: centers[:, index] for index, name in enumerate(geometry["collision_sphere_ids"])},
            geometry,
        ),
        "interaction_candidate_intervals": ici,
        "interval_semantics": "[start, end), nominal route samples",
        "portal_graph": extract_portal_graph(clearance, ee),
        "rejoin_anchors": compute_rejoin_anchors(clearance, ee, ici),
        "rejoin_variant": "first_statically_clear_route_point_after_each_ICI",
        "goal_approach_cone": compute_goal_approach_cone(ee, np.asarray(sample["goal"])),
        "trs": {"available": False, "reason": "C-space samples are not included in this route example."},
    }
    (args.output / "compiled_srir.json").write_text(
        json.dumps(srir, indent=2, allow_nan=False), encoding="utf-8"
    )

    # Select the longest ICI and reuse the original crossing primitive.
    # This is a proposal, not an accepted episode or a G_solve witness.
    window = max(ici, key=lambda row: row["duration"])
    start, end = compute_smart_endpoints(
        ee, {"axis": "y", "path_span": .24}, len(ee), event_window=window
    )
    speed = .18
    motion_steps = max(1, int(np.linalg.norm(end - start) / (speed * DT)))
    target_step = int(round((window["start"] + window["end"]) / 2))
    start_delay = max(0, target_step - motion_steps // 2)
    obstacle_radius = .025
    dynamic = generate_sphere_trace(
        start=start, end=end, speed=speed, total_steps=len(ee),
        start_delay_steps=start_delay, sphere_radius=obstacle_radius, hold_steps=0,
    )
    active = dynamic[:, :, 2] < 1.5
    first_active = int(np.flatnonzero(active[:, 0])[0])
    activation_clearance = float(np.min(
        np.linalg.norm(centers[first_active] - dynamic[first_active, 0], axis=1)
        - radii - obstacle_radius
    ))
    min_dynamic_static = min(
        _signed_point_aabb_distance(point, obstacle, size) - obstacle_radius
        for point in dynamic[active]
        for obstacle, size in zip(obstacles, sizes)
    )
    severity = compute_dynamic_contact_severity(
        robot_centers=centers,
        raw_robot_radii=np.broadcast_to(radii, centers.shape[:2]),
        robot_links=np.asarray([row["link"] for row in geometry["collision_spheres"]]),
        dynamic_centers=dynamic, dynamic_radii=np.asarray([obstacle_radius]),
        dynamic_active=active, time_s=np.arange(len(ee)) * dt,
    )
    np.savez_compressed(
        args.output / "dynamic_proposal.npz", dynamic_centers=dynamic,
        active=active, radius_m=np.asarray([obstacle_radius]), time_s=np.arange(len(ee)) * dt,
    )
    summary = {
        "scope": "partial_stage_example",
        "scene_id": sample["scene_id"],
        "static_templates": len(scene_results),
        "static_templates_passing_geometry_rules": sum(row["valid"] for row in scene_results),
        "separate_static_variants_generated": len(variants),
        "route_samples": len(ee), "robot_spheres": len(radii),
        "minimum_static_route_clearance_m": float(clearance.min()),
        "ici_count": len(ici), "portal_count": srir["portal_graph"]["n_portals"],
        "route_static_rejoin_candidates": len(srir["rejoin_anchors"]),
        "proposal_minimum_static_clearance_m": float(min_dynamic_static),
        "proposal_first_active_robot_clearance_m": activation_clearance,
        "proposal_nominal_contact_metrics": severity,
        "proposal_status": "generated_and_geometrically_measured; not certified by G_solve",
        "not_executed": ["new_reference_route_planning", "C_space_scan", "G_solve", "controller_rollout"],
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
