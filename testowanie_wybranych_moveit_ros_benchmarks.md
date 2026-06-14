# Testing selected planners with `moveit_ros_benchmarks` (by hand)

A **manual, self-contained** walkthrough for task 2: configure, launch and read
`moveit_ros_benchmarks` for a **handful of OMPL planners you pick** — e.g. "RRTConnect vs
RRTstar on one query, 3 runs". There is **no in-repo automation** for this: you build the
scene graphically in RViz and run MoveIt's own benchmark tool. Everything you need is in
this file.

> Implements task-2: *Konfiguracja, uruchomienie i przetestowanie wybranych algorytmów za
> pomocą modułu `moveit_ros_benchmarks`.* Upstream reference:
> <https://moveit.picknik.ai/main/doc/examples/benchmarking/benchmarking_tutorial.html>.

The automated, cross-framework comparison (MoveIt **and** cuRobo, uniform metrics) is the
separate custom harness — `mb-benchmark run` / `make benchmark`. This page is only the
standard MoveIt-internal benchmark, done manually.

Everything below runs **inside the `ros` container**:

```bash
docker compose -f docker/docker-compose.yml run --rm ros bash
```

---

## How the module works (30-second model)

`moveit_run_benchmark` does **not** invent problems. It reads, from a **warehouse**
(a `.sqlite` file here):

- one **planning scene** (the obstacle world), selected by `scene_name`;
- a set of **queries / states** (start + goal), selected by regex.

For every `(planner × query)` pair it plans `runs` times and logs planning time, path
length, success, etc. to `.log` files. So "which algorithms" and "which states" are *both*
just config — you don't need all the planners and you don't need random states.

```
RViz (build scene + start/goal, Save to warehouse)
        │
        ▼
warehouse.sqlite ─► moveit_run_benchmark ─► *.log ─► moveit_benchmark_statistics.py ─► benchmark.db ─► Planner Arena
   (scene+states)     (your benchmark.yaml                                              (or just read
                       picks planners+query)                                             the .log text)
```

---

## Available planners

The planners you may list are the OMPL configs shipped in **`ur_moveit_config`'s
`ompl_planning.yaml`** — a name only works if it has a config block there. Print the exact
list available on your install:

```bash
# inside the ros container:
grep -oE '^[A-Za-z0-9]+kConfigDefault' \
  "$(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/config/ompl_planning.yaml" \
  | sort -u
```

The standard MoveIt/OMPL set (what you'll typically find) — pick from these:

| Planner | Type | Notes — good for |
|---|---|---|
| **RRTConnect** | feasible (bi-directional RRT) | the default; fast, finds *a* path. Best baseline. |
| RRT | feasible | classic single-tree RRT; slower than RRTConnect. |
| EST | feasible | expansive-space tree. |
| SBL | feasible (bi-directional) | single-query, lazy collision checking. |
| KPIECE / LBKPIECE / BKPIECE | feasible | cell-based coverage; good in cluttered scenes. |
| PRM | feasible (roadmap) | multi-query; builds a graph. |
| BiTRRT | feasible (cost-aware) | transition-based RRT, bi-directional. |
| **RRTstar** | **optimizing** | asymptotically optimal; shorter paths if given time. |
| PRMstar | optimizing roadmap | optimal variant of PRM. |
| **BITstar** | optimizing | batch informed trees; often best path-quality/time. |
| LBTRRT / TRRT | (near-)optimizing | lower-bound / transition RRT. |

For a teaching comparison, the most informative contrast is **one feasible planner vs one
optimizing planner**, e.g. `RRTConnect` (fast, any path) vs `RRTstar`/`BITstar` (slower,
shorter/smoother path). That's the comparison the example below is set up for. A reasonable
small selection: `RRTConnect, RRT, RRTstar, BITstar, PRM`.

---

## Step 1 — Build the scene + start/goal graphically in RViz

No scenario files are loaded — you draw the obstacle world by hand and store it.

1. **Bring up move_group + RViz** (UR5e mock hw + MoveIt). This is task-1's sim:

   ```bash
   make sim
   ```

   **The warehouse must be enabled** so RViz can Save scenes/states. move_group needs:
   `warehouse_plugin = warehouse_ros_sqlite::DatabaseConnection` and
   `warehouse_host = /workspace/results/moveit_benchmarks/warehouse.sqlite`. If your sim
   launch doesn't already set these, pass them as launch args / params for your UR moveit
   launch (arg names vary by UR driver version — verify on your box). In the RViz
   **MotionPlanning** panel the **Stored Scenes / Stored States** tabs must show *Connected*.

2. **Add obstacles.** MotionPlanning panel → **Scene Objects** tab → add primitive(s)
   (e.g. a box), position them in front of the arm so a straight path is blocked, then
   **Publish** the scene.

3. **Save the scene to the warehouse.** **Stored Scenes** tab → give it a name
   (e.g. `single_box`) → **Save**. That name is your `scene_name`.

4. **Store a start and a goal state.** Drag the interactive marker to a start pose →
   **Stored States** → Save as e.g. `start`. Repeat for a goal pose → `goal`. (A benchmark
   "query" is a start state + a goal constraint; storing named states is the simplest way
   to give the benchmark a fixed, non-random problem.)

> Mongo isn't packaged for Jazzy/Humble, so the backend is **SQLite** — the warehouse is
> just the `.sqlite` file above, no DB server to run.

---

## Step 2 — Write the benchmark config (pick the algorithms)

Create `/workspace/benchmark.yaml`. Three knobs keep it a quick manual test:

- **`planners:`** — list only the ones you're testing (the whole point).
- **`runs:`** — small (e.g. `3`) so it finishes in seconds.
- **`scene_name` / regexes** — pin to the scene + states you stored, so the problem is
  fixed, not random.

```yaml
# /workspace/benchmark.yaml  — RRTConnect vs RRTstar, one scene, 3 runs.
# The whole tree MUST stay under `/**: ros__parameters:` (rcl errors otherwise).
/**:
  ros__parameters:
    benchmark_config:

      warehouse:
        host: "/workspace/results/moveit_benchmarks/warehouse.sqlite"
        port: 0
        scene_name: "single_box"          # the scene you Saved in RViz

      parameters:
        name: "manual_rrtconnect_vs_rrtstar"
        runs: 3                            # repetitions per (planner, query)
        group: "ur_manipulator"
        timeout: 10.0                      # seconds per planning attempt
        output_directory: "/workspace/results/moveit_benchmarks/"
        queries_regex: ".*"                # match the stored states/queries
        start_states_regex: ".*"
        goal_constraints_regex: ".*"

      planning_pipelines:
        pipelines: ["ompl"]
        ompl:
          name: "ompl"
          planners:
            - "RRTConnect"                 # feasible — finds a path fast
            - "RRTstar"                    # optimizing — shorter path, slower
```

Swap/extend the planner list for any names from the table above. To test a **single**
algorithm, leave one entry.

---

## Step 3 — Run `moveit_run_benchmark`

`moveit_run_benchmark` needs the UR5e MoveIt config (robot model, SRDF, kinematics, OMPL
planner configs) loaded as parameters alongside `benchmark.yaml`. The clean way to assemble
that is a tiny **throwaway** launch file — save it as `/workspace/run_benchmark.launch.py`:

```python
# /workspace/run_benchmark.launch.py  — throwaway; not part of the repo.
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

UR_TYPE = "ur5e"
BENCHMARK_YAML = "/workspace/benchmark.yaml"
WAREHOUSE = "/workspace/results/moveit_benchmarks/warehouse.sqlite"


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"),
            mappings={"ur_type": UR_TYPE, "name": "ur", "tf_prefix": ""})
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": UR_TYPE})
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .joint_limits(Path("config") / "joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(
            package="moveit_ros_benchmarks",
            executable="moveit_run_benchmark",
            output="screen",
            parameters=[
                BENCHMARK_YAML,
                moveit_config.to_dict(),
                {"warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                 "warehouse_host": WAREHOUSE},
            ],
        )
    ])
```

Run it:

```bash
# inside the ros container, from /workspace:
ros2 launch /workspace/run_benchmark.launch.py
```

Watch the console: it prints each planner/query as it runs and where it wrote the `.log`.
(You can leave RViz/move_group from Step 1 running or stop it — the benchmark loads its own
robot model from the launch file above; only the **warehouse file** needs to exist.)

---

## Step 4 — Read the results

Logs land in `output_directory` → on the host: `results/moveit_benchmarks/*.log`.

**A. Just read the log** — plain text with per-run planning time and success/path metrics.
Fastest manual sanity check.

**B. Aggregate → Planner Arena** (plots comparing the planners). The stats script ships
with `moveit_ros_benchmarks`:

```bash
STATS="$(ros2 pkg prefix moveit_ros_benchmarks)/lib/moveit_ros_benchmarks/moveit_benchmark_statistics.py"
python3 "$STATS" /workspace/results/moveit_benchmarks/*.log \
    --database /workspace/results/moveit_benchmarks/benchmark.db
```

Then open `results/moveit_benchmarks/benchmark.db` at <https://plannerarena.org> (or the
local Planner Arena viewer): planning-time / path-length / success curves, one series per
planner you selected.

---

TODO Jeżeli chcemy tylko ze bylo odpaleone ale bez grafow co i tak bd imo okey 
ale mozna by odpalic te same scenariusze wiec ogolnie polacalbym Ci odpalic po kolei rzecyz w sensie 
BUILD i potem bash scripts/run_harness.sh ale tylko pierwszy kros sie musi zrobic i weteyd w scenarios/generated bd dokladnie te sciezki kotre zostaly uzyte do tych pozniejszych planowan i dac te wykresiki do raportu jako porwnanie 

In the **MotionPlanning** panel → **Context** tab → **OMPL** → pick the planner from the
dropdown → set a goal → **Plan**. Fastest "does RRTstar even solve this?" check, but it
produces no logs/metrics — use `moveit_run_benchmark` (above) for actual numbers.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| RViz **Stored Scenes** shows *not connected* / can't Save | move_group launched without the warehouse params. Set `warehouse_plugin` + `warehouse_host` (Step 1). |
| `Failed to load scene <name>` / hard exit | Warehouse empty or `scene_name` mismatch. Re-do Step 1 and check `host` points at the right `.sqlite`. |
| `No queries matched` | The regexes don't match what you stored. Confirm the stored scene/state names; widen to `.*`. |
| Planner name has no effect / errors | That planner isn't in `ur_moveit_config`'s `ompl_planning.yaml`. List the real names (see *Available planners*). |
| `Cannot have a value before ros__parameters` | The whole tree must stay under `/**: ros__parameters:` — don't unindent `benchmark_config`. |
| Logs empty / no `.log` | Check `output_directory` is writable and bind-mounted (`/workspace/results/...` → host `results/`). |

> Task 2 is intentionally **non-blocking**: the custom harness (`mb-benchmark run`)
> independently re-measures the same MoveIt planners on the same scenarios, so the
> comparison is covered even if this manual warehouse/RViz step is fiddly.
