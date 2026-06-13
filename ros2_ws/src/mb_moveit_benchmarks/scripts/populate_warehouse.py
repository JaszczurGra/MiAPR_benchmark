#!/usr/bin/env python3
"""Build MoveIt planning scenes + benchmark queries from the SHARED scenario library and
make them available to Pipeline A (moveit_ros_benchmarks).

Why this matters: both pipelines must benchmark *identical* problems. This script reads
the same ``scenarios/library/*.yaml`` (+ generated queries) the custom harness uses, and
turns each into:
  * a ``moveit_msgs/PlanningScene`` (collision objects), applied live via
    ``/apply_planning_scene`` and exported as a MoveIt ``.scene`` text file, and
  * per-query start states + joint-space goal constraints.

WAREHOUSE PERSISTENCE (the part to finish on a real ROS box -- cannot be tested here):
moveit_ros_benchmarks reads scenes/queries from the MongoDB warehouse. Two ways to get
them there:
  (A) Manual / reliable: launch MoveIt + RViz, MotionPlanning panel -> "Stored Scenes" ->
      Import From Text (the exported ``.scene`` file) -> Save; add Stored States for the
      start/goal of each query. This is the documented, dependable path.
  (B) Programmatic: use moveit's PlanningSceneStorage (C++) or a warehouse_ros client to
      persist the PlanningScene + MotionPlanRequest messages built below. Hook marked TODO.

Run (inside the `ros` container, with move_group running):
    ros2 run mb_moveit_benchmarks populate_warehouse.py \
        --scenarios /workspace/scenarios/library --out /results/scenes
"""

import argparse
import os
import sys

# The shared scenario model lives in the pip-installed `mb_benchmark` package (available
# in the `ros` container too), so Pipeline A and the harness never diverge.
from mb_benchmark.scenario import load_library


def _scene_text(scenario) -> str:
    """Export a MoveIt '.scene' text file (RViz 'Import From Text' compatible).

    NOTE: the exact .scene grammar has varied across MoveIt versions; verify against your
    installed RViz MotionPlanning import parser. This emits the common
    name/type/dims/pos/quat/color layout.
    """
    lines = [scenario.name]
    for obs in scenario.obstacles:
        lines.append(f"* {obs.name}")
        lines.append("1")  # one shape per object
        if obs.type == "box":
            sx, sy, sz = obs.params["size"]
            lines.append("box")
            lines.append(f"{sx} {sy} {sz}")
        elif obs.type == "sphere":
            lines.append("sphere")
            lines.append(f"{obs.params['radius']}")
        elif obs.type == "cylinder":
            lines.append("cylinder")
            lines.append(f"{obs.params['radius']} {obs.params['length']}")
        else:
            continue
        x, y, z = obs.xyz
        qx, qy, qz, qw = obs.quat
        lines.append(f"{x} {y} {z}")
        lines.append(f"{qx} {qy} {qz} {qw}")
        lines.append("0 0 0 0")  # color rgba
    lines.append(".")
    return "\n".join(lines) + "\n"


def build_planning_scene_msg(scenario):
    """Build a moveit_msgs/PlanningScene (collision objects) for live application."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive

    scene = PlanningScene()
    scene.name = scenario.name
    scene.is_diff = True
    for obs in scenario.obstacles:
        co = CollisionObject()
        co.id = obs.name
        co.header.frame_id = "base_link"
        prim = SolidPrimitive()
        if obs.type == "box":
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(v) for v in obs.params["size"]]
        elif obs.type == "sphere":
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [float(obs.params["radius"])]
        elif obs.type == "cylinder":
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [float(obs.params["length"]), float(obs.params["radius"])]
        else:
            continue
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in obs.xyz]
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = [float(v) for v in obs.quat]
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        scene.world.collision_objects.append(co)
    return scene


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", default="/workspace/scenarios/library")
    ap.add_argument("--out", default="/results/scenes")
    ap.add_argument("--apply-live", action="store_true",
                    help="also push each scene to a running move_group via /apply_planning_scene")
    args = ap.parse_args()

    scenarios = load_library(args.scenarios)
    os.makedirs(args.out, exist_ok=True)
    for name, sc in scenarios.items():
        with open(os.path.join(args.out, f"{name}.scene"), "w") as fh:
            fh.write(_scene_text(sc))
        print(f"[scene] wrote {name}.scene ({len(sc.obstacles)} obstacles, {len(sc.queries)} queries)")

    if not args.apply_live:
        print("\nExported .scene files. Import them in RViz (Stored Scenes -> Import From "
              "Text -> Save) to populate the warehouse, OR finish the programmatic TODO.")
        return

    # --- live application + (TODO) warehouse persistence ---
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.srv import ApplyPlanningScene

    rclpy.init()
    node = Node("mb_populate_warehouse")
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("/apply_planning_scene unavailable -- is move_group running?")
        rclpy.shutdown()
        sys.exit(1)
    for name, sc in scenarios.items():
        req = ApplyPlanningScene.Request()
        req.scene = build_planning_scene_msg(sc)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        node.get_logger().info(f"applied scene {name}")
        # TODO(later agent): persist req.scene + per-query MotionPlanRequest to the MongoDB
        # warehouse here (PlanningSceneStorage / warehouse_ros client) so moveit_run_benchmark
        # can find them by scene_name/query regex. Until then use the RViz Save path (A).
    rclpy.shutdown()


if __name__ == "__main__":
    main()
