"""2v2 PettingZoo multi-agent env — STUB.

Implement when Stage.SELF_PLAY_2V2 is ready:
  - Subclass pettingzoo.ParallelEnv
  - Add 4 agents: robot_0, robot_1 (team A), robot_2, robot_3 (team B)
  - Extend PyPhysics to handle N robots and inter-robot collisions
  - Use shared-field-frame observations with per-agent egocentric transform
  - Self-play: swap agent assignment each episode
"""


class BuckyTeamEnv:
    """STUB: 2v2 self-play env. Not yet implemented."""

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            "BuckyTeamEnv is not yet implemented. "
            "See Stage.SELF_PLAY_2V2 in curriculum.py for the roadmap."
        )
