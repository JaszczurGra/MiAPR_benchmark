"""mb_benchmark — cross-framework manipulator motion-planning benchmark harness.

Importing this package is intentionally lightweight and has NO ROS / torch / cuRobo
dependencies, so the offline core (scenarios, metrics, analysis, synthetic planners)
can be used and tested without any of that installed. Heavy, environment-specific
imports live *inside* the MoveIt / cuRobo adapter ``setup()`` methods.
"""

__version__ = "0.1.0"

__all__ = ["__version__"]
