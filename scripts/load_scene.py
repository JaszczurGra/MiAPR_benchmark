#!/usr/bin/env python3
"""Load a scenario YAML into the running MoveIt planning scene (RViz)."""
import sys, time, yaml, glob, os, rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def make_co(obs):
    co = CollisionObject()
    co.header.frame_id = 'world'
    co.id = obs['name']
    co.operation = CollisionObject.ADD
    p = Pose(); p.orientation.w = 1.0
    xyz = obs['pose']['xyz']
    p.position.x, p.position.y, p.position.z = xyz
    co.primitive_poses = [p]
    prim = SolidPrimitive()
    if obs['type'] == 'box':
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(obs['size'])
    elif obs['type'] == 'cylinder':
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [obs['length'], obs['radius']]
    elif obs['type'] == 'sphere':
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [obs['radius']]
    co.primitives = [prim]
    return co


def main():
    yaml_path = sys.argv[1]
    with open(yaml_path) as f:
        scenario = yaml.safe_load(f)

    rclpy.init()
    node = Node('scene_loader')
    pub = node.create_publisher(PlanningScene, '/planning_scene', 10)
    time.sleep(1.0)

    # Clear scene - wczytaj nazwy ze wszystkich yamli w library
    ps_clear = PlanningScene(); ps_clear.is_diff = True
    lib_dir = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), '..', 'library')
    for yf in glob.glob(os.path.join(lib_dir, '*.yaml')):
        with open(yf) as f2:
            s = yaml.safe_load(f2)
        for obs in s.get('world', {}).get('obstacles', []):
            co = CollisionObject()
            co.header.frame_id = 'world'
            co.id = obs['name']
            co.operation = CollisionObject.REMOVE
            ps_clear.world.collision_objects.append(co)
    pub.publish(ps_clear); time.sleep(0.5)

    # Zaladuj nowe obiekty
    ps = PlanningScene(); ps.is_diff = True
    obstacles = scenario.get('world', {}).get('obstacles', [])
    for obs in obstacles:
        ps.world.collision_objects.append(make_co(obs))
    pub.publish(ps); time.sleep(0.5)

    print(f"Loaded {len(obstacles)} obstacles from: {yaml_path}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()