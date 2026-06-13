"""Planner adapters and the registry that makes the harness extensible (task 5).

Importing this package registers every adapter that *can* be constructed in the current
environment. The MoveIt and cuRobo adapter modules import cleanly even without ROS /
torch installed -- their heavy imports happen inside ``setup()`` -- so their planner
names always appear in the registry; calling ``setup()`` without the dependency raises
a clear, actionable error.
"""

from .base import (  # noqa: F401
    PLANNER_REGISTRY,
    PlannerAdapter,
    PlanResult,
    available_planners,
    get_adapter,
    register,
)

# Each module self-registers on import. Order is irrelevant.
from . import synthetic_adapter  # noqa: F401,E402
from . import moveit_adapter  # noqa: F401,E402
from . import curobo_adapter  # noqa: F401,E402
from . import template_adapter  # noqa: F401,E402
