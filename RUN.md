# RUN — step by step

Two phases: **(A) offline**, runnable right now on the host with only Python; and
**(B) full stack**, runnable once Docker + NVIDIA container toolkit are installed.

---

## A. Offline (no ROS, no GPU, no Docker) — works today

### A1. Set up and test
```bash
cd /home/jaszczurgra/Documents/University/MiAPR_benchmark
python -m venv .venv
source .venv/bin/activate
pip install -e ./benchmark pytest
pytest benchmark/tests -q          # expect: 30 passed
```
(or just `make offline`)

### A2. Run the offline demo (end-to-end pipeline on synthetic planners)
```bash
mb-benchmark demo --out results/demo
```
Produces:
- `results/demo/report/report.md` — comparison table
- `results/demo/report/plots/*.png` — success rate, planning time, path length, smoothness, clearance
- `results/demo/report/metrics.csv`, `summary.csv`

### A3. Use the pipeline piecemeal
```bash
mb-benchmark list-planners                                   # what's registered
mb-benchmark generate --out scenarios/generated --num 20     # seeded queries
mb-benchmark run --scenarios scenarios/generated --no-autogen \
    --planners "synthetic:bitstar,straightline" --out results
mb-benchmark metrics --raw results/raw --scenarios scenarios/generated --no-autogen \
    --out results/metrics.csv
mb-benchmark report --metrics results/metrics.csv --out results/report
```

> `straightline` is a real, dependency-free baseline planner (joint-space line, succeeds
> only if collision-free). It’s also the worked example for adding your own planner.

---

## B. Full stack (ROS 2 + cuRobo) — after install

### B0. Host prerequisites (one time)
Docker + Compose are already present. Install the **NVIDIA container toolkit** so the GPU
is visible inside Docker (needed for cuRobo):
```bash
# Arch Linux (AUR):
yay -S nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
# verify:
docker run --rm --gpus all nvidia/cuda:12.1.1-base-ubuntu22.04 nvidia-smi
```
For RViz GUIs: `xhost +local:`.

### B1. Build the images
```bash
make build       # docker compose -f docker/docker-compose.yml build
```

### B2. TASK 1 — UR5e simulation
```bash
make sim         # UR5e (mock hardware) + MoveIt + RViz
```
You should see the UR5e in RViz and be able to plan with the MotionPlanning panel.

### B3. TASK 2 — Pipeline A (moveit_ros_benchmarks → Planner Arena)
```bash
make benchmark-moveit
# -> results/moveit_benchmarks/benchmark.db  (upload to https://plannerarena.org)
```
See `docs/02` (the warehouse-population step needs finishing on a real box — `HANDOFF.md`).

### B4. TASKS 3+4 — Pipeline B (harness over MoveIt + cuRobo)
```bash
make benchmark
# 1) generates identical seeded queries (scenarios/generated)
# 2) runs MoveIt planners in the `ros` container
# 3) runs cuRobo in the `curobo` container (GPU)
# 4) scores all trajectories uniformly -> results/report/report.md
```
Tune with env vars: `RUNS=25 TIMEOUT=10 MOVEIT_PLANNERS=moveit:RRTConnect,moveit:BITstar make benchmark`.

### B5. TASK 5 — add a planner
Copy `benchmark/mb_benchmark/adapters/template_adapter.py`, implement `setup`/`plan`,
`register(...)`, import it in `adapters/__init__.py`. See `docs/05`. It then appears in
`list-planners` and flows through the same metrics/report automatically.

---

## Interactive shells
```bash
make shell-ros       # bash inside the ROS 2 container
make shell-curobo    # bash inside the cuRobo (GPU) container
```
