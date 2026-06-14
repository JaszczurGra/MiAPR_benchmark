"""Load the harness configuration (``config/planners.yaml``).

This is the single, environment-independent source for the cross-framework run
defaults: how many ``runs`` per query, the per-plan ``timeout``, how many queries
("maps") per scenario to generate (``num``), and the named
planner *groups* (``moveit`` / ``curobo`` / ``baselines``) that the CLI expands
with ``@group`` (e.g. ``--planners @moveit``).

It lives in the offline core (pure Python, PyYAML only) so it is testable without
ROS/GPU and so *every* caller -- ``mb-benchmark`` run directly, or via
``run_harness.sh`` -- resolves the same values. Precedence, highest first::

    explicit --runs/--timeout/--planners  >  config/planners.yaml  >  built-in fallback
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

# Built-in fallback, used when the file (or a key) is absent. Mirrors the historical
# argparse defaults so behaviour is unchanged when no config file is present.
_FALLBACK_RUNS = 5
_FALLBACK_TIMEOUT = 10.0
_FALLBACK_NUM = 20

CONFIG_ENV = "MB_PLANNERS_CONFIG"  # optional override for the config path
_REL = Path("config") / "planners.yaml"


@dataclass
class HarnessConfig:
    runs: int = _FALLBACK_RUNS
    timeout: float = _FALLBACK_TIMEOUT
    num: int = _FALLBACK_NUM
    groups: Dict[str, List[str]] = field(default_factory=dict)
    path: Optional[Path] = None  # where it was loaded from (None == built-in fallback)

    def group(self, name: str) -> List[str]:
        if name not in self.groups:
            known = ", ".join(sorted(self.groups)) or "(none)"
            where = self.path or "built-in defaults"
            raise KeyError(
                f"planner group {name!r} is not defined in {where}; known groups: {known}"
            )
        return list(self.groups[name])


def find_config(explicit: Optional[str] = None) -> Optional[Path]:
    """Locate ``config/planners.yaml``. Order: explicit path, ``$MB_PLANNERS_CONFIG``,
    then walk up from the CWD and from this module. Returns ``None`` if not found.

    The CWD walk is what makes it work inside the containers: ``working_dir`` is
    ``/workspace`` there, but the package is pip-installed into site-packages, so a
    ``__file__``-relative lookup alone would miss ``/workspace/config/planners.yaml``.
    """
    if explicit:
        p = Path(explicit)
        if not p.is_file():
            raise FileNotFoundError(f"--config {explicit}: no such file")
        return p
    env = os.environ.get(CONFIG_ENV)
    if env and Path(env).is_file():
        return Path(env)
    for base in (Path.cwd(), Path(__file__).resolve().parent):
        for d in (base, *base.parents):
            cand = d / _REL
            if cand.is_file():
                return cand
    return None


def load_harness_config(explicit: Optional[str] = None) -> HarnessConfig:
    """Read the config into a :class:`HarnessConfig`. Missing file -> built-in fallback."""
    path = find_config(explicit)
    if path is None:
        return HarnessConfig()
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    defaults = data.get("defaults", {}) or {}
    groups_raw = data.get("harness", {}) or {}
    groups = {str(k): [str(x) for x in (v or [])] for k, v in groups_raw.items()}
    return HarnessConfig(
        runs=int(defaults.get("runs", _FALLBACK_RUNS)),
        timeout=float(defaults.get("timeout", _FALLBACK_TIMEOUT)),
        num=int(defaults.get("num", _FALLBACK_NUM)),
        groups=groups,
        path=path,
    )
