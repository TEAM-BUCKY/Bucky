"""Bisector approach — the GeoGebra drive formula.

User frame (cm, robot O at the origin, +y towards the opponent goal):
    B = ball
    C = B - (0, behind_dist)                       point behind the ball
    g = AngleBisector(C, B, O)                     bisector of ∠CBO
    r = (behind_dist + |BO|) / 2
    A = Intersect(circle(B, r), g, 2) = B + r · unit(û(B→C) + û(B→O))

Driving to A each frame swings the robot around the ball and settles it at C, behind the ball.
"""
from bucky.lab import DriveModule, O, Param, Vec2, register


@register
class BisectorDrive(DriveModule):
    """Drive to A on the bisector of ∠CBO, at radius (behind_dist + |BO|) / 2 around the ball."""

    name = "bisector"
    params = {
        "behind_dist": Param(20.0, 5.0, 60.0, 1.0, "Distance of C behind the ball (cm)"),
    }

    def target(self, ctx):
        B = ctx.ball
        if B is None:                       # ball not seen this frame → stand still
            return None
        C = B - Vec2(0.0, self.p.behind_dist)
        ctx.mark("C", C)
        BO = O - B
        if BO.norm() < 1e-6:
            return C
        s = (C - B).unit() + BO.unit()      # direction of the angle bisector g
        if s.norm() < 1e-6:                 # robot exactly in front of the ball: go around right
            s = Vec2(1.0, 0.0)
        r = (self.p.behind_dist + BO.norm()) / 2
        return B + s.unit() * r
