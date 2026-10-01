"""Bucky Lab: drop-in Python modules (drive formulas, sensor code, …) tested across every start
position in the real training physics.

Quick start — create ``bucky/lab/user/my_drive.py``::

    from bucky.lab import DriveModule, Param, Vec2, register

    @register
    class StraightAtBall(DriveModule):
        \"\"\"Drive straight at the ball.\"\"\"
        name = "straight"

        def target(self, ctx):
            return ctx.ball          # ball position, user frame (cm, robot at origin)

then ``uv run python scripts/lab.py sweep straight`` or open the Lab page in the dashboard.
"""
from bucky.lab.frames import O, Vec2
from bucky.lab.modules import DriveCommand, DriveModule, LabModule, RobotView
from bucky.lab.params import Param
from bucky.lab.registry import discover, get_module, list_modules, register
from bucky.lab.sensors import Sensor, TruthState, register_sensor

__all__ = [
    "O", "Vec2", "DriveCommand", "DriveModule", "LabModule", "RobotView", "Param",
    "discover", "get_module", "list_modules", "register", "Sensor", "TruthState",
    "register_sensor",
]
