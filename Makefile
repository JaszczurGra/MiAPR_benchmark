# MiAPR motion-planning benchmark — task runner.
# Offline targets need only Python. The rest need Docker (+ nvidia-container-toolkit for GPU).

COMPOSE := docker compose -f docker/docker-compose.yml
VENV    := .venv/bin/activate

.PHONY: help offline test demo build sim benchmark-moveit benchmark report shell-ros shell-curobo clean

help:
	@echo "Offline (no ROS/GPU):"
	@echo "  make offline    - create venv, install harness, run tests"
	@echo "  make test       - run the offline test suite"
	@echo "  make demo       - offline end-to-end demo (synthetic planners) -> results/demo"
	@echo "Containers (need Docker; GPU needs nvidia-container-toolkit):"
	@echo "  make build           - build the ros + curobo images"
	@echo "  make sim             - TASK 1: UR5e mock-hw sim + MoveIt + RViz"
	@echo "  make benchmark-moveit - TASK 2: moveit_ros_benchmarks -> Planner Arena"
	@echo "  make benchmark       - TASKS 3+4: harness over MoveIt + cuRobo -> results/report"
	@echo "  make shell-ros / shell-curobo - interactive shells"

offline:
	bash scripts/setup_offline.sh

test:
	. $(VENV) && python -m pytest benchmark/tests -q

demo:
	. $(VENV) && mb-benchmark demo --out results/demo

build:
	$(COMPOSE) build

sim:
	bash scripts/run_simulation.sh

benchmark-moveit:
	bash scripts/run_moveit_benchmark.sh

benchmark:
	bash scripts/run_harness.sh

report:
	. $(VENV) && mb-benchmark report --raw results/raw --scenarios scenarios/generated --no-autogen --out results/report

shell-ros:
	$(COMPOSE) run --rm ros bash

shell-curobo:
	$(COMPOSE) run --rm curobo bash

clean:
	rm -rf results/ scenarios/generated/ benchmark/*.egg-info benchmark/build
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
