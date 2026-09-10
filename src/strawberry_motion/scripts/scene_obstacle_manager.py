#!/usr/bin/env python3
"""Collision-world and neighbor-obstacle state for harvest planning."""

import json

import numpy as np
from curobo.geom.types import Cuboid, Sphere, WorldConfig

from harvest_motion_params import NEIGHBOR_SPHERE_RADIUS_M


class SceneObstacleManager:
    """Owns dynamic cuboids, neighbor spheres, and scene-position callbacks."""

    def __init__(self, node, runtime_log, motion_gen, static_cuboids):
        self.node = node
        self.runtime_log = runtime_log
        self.motion_gen = motion_gen
        self.static_cuboids = list(static_cuboids)
        self.dynamic_cuboids = []
        self.neighbor_spheres = []
        self.registered_neighbor_positions = []
        self.scene_positions = []

    def _log(self):
        return self.node.get_logger()

    def set_motion_gen(self, motion_gen):
        self.motion_gen = motion_gen

    def world_state(self):
        return self.static_cuboids, self.dynamic_cuboids, self.neighbor_spheres

    def update_curobo_world(self, reason="manual"):
        cuboids = self.static_cuboids + self.dynamic_cuboids
        self.motion_gen.update_world(
            WorldConfig(cuboid=cuboids, sphere=self.neighbor_spheres))
        self._log().info(
            f"World updated ({reason}): static={len(self.static_cuboids)} "
            f"dynamic={len(self.dynamic_cuboids)} "
            f"neighbor_spheres={len(self.neighbor_spheres)}")
        self.runtime_log.log(
            "collision_world_update",
            reason=reason,
            cuboids=[{"name": c.name, "pose": c.pose, "dims": c.dims} for c in cuboids],
            neighbor_spheres=[
                {"name": s.name, "pose": s.pose, "radius": s.radius}
                for s in self.neighbor_spheres
            ],
        )

    def obstacles_from_json(self, text: str):
        data = json.loads(text)
        cuboids = []
        for obj in data:
            cuboids.append(Cuboid(
                name=obj["name"],
                pose=[*obj["pos"], 1, 0, 0, 0],
                dims=obj.get("dims", [0.05, 0.05, 0.05]),
            ))
        self.dynamic_cuboids = cuboids
        self.update_curobo_world("dynamic obstacles")

    def update_scene_positions_from_flat_array(self, data):
        self.scene_positions = [
            np.array([data[i], data[i+1], data[i+2]])
            for i in range(0, len(data) - 2, 3)
        ]
        self.runtime_log.log(
            "scene_positions_received",
            positions_m=self.scene_positions,
        )

    def register_neighbor_obstacles(self, target_pos: np.ndarray) -> None:
        spheres = []
        registered_positions = []
        # [FIX 2026-09-10] 자기-제외 임계를 여유 있게. 호출자가 raw 검출 좌표를
        # 넘기면 거리가 0 이라 원래 안전하지만, bias 가 붙은 좌표가 들어와도
        # 경계에서 갈리지 않도록 z bias(35mm)+여유까지 덮는다.
        # 이웃 딸기는 최소 60mm 떨어뜨려 배치하므로 50mm 임계는 이웃을 지우지 않는다.
        self_exclude_m = 0.050
        skipped = 0
        for i, pos in enumerate(self.scene_positions):
            if np.linalg.norm(pos - target_pos) < self_exclude_m:
                skipped += 1
                continue
            spheres.append(Sphere(
                name=f"neighbor_{i}",
                pose=[float(pos[0]), float(pos[1]), float(pos[2]), 1.0, 0.0, 0.0, 0.0],
                radius=NEIGHBOR_SPHERE_RADIUS_M,
            ))
            registered_positions.append(np.array(pos, dtype=float))
        self.neighbor_spheres = spheres
        self.registered_neighbor_positions = registered_positions
        self.update_curobo_world("neighbor obstacles registered")
        self._log().info(
            f"Registered {len(spheres)} neighbor sphere obstacle(s); "
            f"self-excluded {skipped} within {self_exclude_m*1000:.0f}mm of target")

    def clear_neighbor_obstacles(self) -> None:
        had_neighbors = bool(self.neighbor_spheres or self.registered_neighbor_positions)
        self.neighbor_spheres = []
        self.registered_neighbor_positions = []
        if had_neighbors:
            self.update_curobo_world("neighbor obstacles cleared")
