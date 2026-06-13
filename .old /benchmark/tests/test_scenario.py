from pathlib import Path

import numpy as np

from mb_benchmark.scenario import (
    Obstacle,
    Query,
    Scenario,
    load_library,
    load_scenario,
    save_scenario,
)

REPO = Path(__file__).resolve().parents[2]
LIBRARY = REPO / "scenarios" / "library"


def test_library_loads():
    lib = load_library(LIBRARY)
    assert {"empty", "single_box", "shelf", "narrow_passage", "cluttered", "table_pick"} <= set(lib)
    shelf = lib["shelf"]
    assert shelf.robot == "ur5e"
    assert len(shelf.obstacles) >= 4
    for obs in shelf.obstacles:
        assert obs.type in {"box", "sphere", "cylinder", "mesh"}
        assert obs.xyz.shape == (3,)
        assert obs.quat.shape == (4,)


def test_obstacle_roundtrip():
    o = Obstacle.from_dict(
        {"type": "box", "name": "b", "size": [1, 2, 3], "pose": {"xyz": [1, 2, 3], "rpy": [0, 0, 0]}}
    )
    d = o.to_dict()
    o2 = Obstacle.from_dict(d)
    assert np.allclose(o.xyz, o2.xyz)
    assert o2.params["size"] == [1, 2, 3]


def test_scenario_save_load(tmp_path):
    sc = Scenario(
        name="t", robot="ur5e",
        obstacles=[Obstacle.from_dict({"type": "sphere", "name": "s", "radius": 0.1, "pose": {"xyz": [0, 0, 1]}})],
        queries=[Query(id=0, start=np.zeros(6), goal_joint=np.ones(6),
                       goal_pose={"xyz": [0.1, 0.2, 0.3], "quat": [0, 0, 0, 1]})],
        generation={"seed": 1},
    )
    p = tmp_path / "t.yaml"
    save_scenario(sc, p)
    sc2 = load_scenario(p)
    assert sc2.name == "t"
    assert len(sc2.queries) == 1
    assert np.allclose(sc2.queries[0].goal_joint, np.ones(6))
