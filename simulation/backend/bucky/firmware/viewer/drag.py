"""Mouse interaction on the field panel: drag the ball or the robot, scroll to rotate the robot,
right-click to drop the ball. Positions are written straight into the physics."""
from __future__ import annotations

import math

import numpy as np
import pygame

from bucky.firmware import frames
from bucky.firmware.viewer.layout import SCALE, field_to_screen, in_field_panel, screen_to_field
from bucky.game.field import ROBOT_RADIUS

HIT_RADIUS_PX = 15


class DragController:
    def __init__(self) -> None:
        self.dragging: str | None = None

    def handle_event(self, event, world) -> None:
        ph = world.physics
        if event.type == pygame.MOUSEBUTTONDOWN and in_field_panel(*event.pos):
            if event.button == 1:
                self.dragging = self._hit(event.pos, world)
            elif event.button == 3:
                ph.place_ball(self._sim_pos(event.pos))
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = None
        elif event.type == pygame.MOUSEMOTION and self.dragging and in_field_panel(*event.pos):
            if self.dragging == "ball":
                ph.place_ball(self._sim_pos(event.pos))
            else:
                ph.place_robot("a", self._sim_pos(event.pos))
        elif event.type == pygame.MOUSEWHEEL:
            s = ph.state_a()
            rx, ry = field_to_screen(*frames.sim_to_fw_pos(s.robot_pos))
            mx, my = pygame.mouse.get_pos()
            if math.hypot(mx - rx, my - ry) < ROBOT_RADIUS * 100 * SCALE * 3:
                # scroll up = counter-clockwise
                ph.place_robot("a", s.robot_pos, s.robot_heading + math.radians(5) * event.y)

    @staticmethod
    def _sim_pos(pos) -> np.ndarray:
        return frames.fw_to_sim_pos(screen_to_field(*pos))

    @staticmethod
    def _hit(pos, world) -> str | None:
        s = world.physics.state_a()
        bx, by = field_to_screen(*frames.sim_to_fw_pos(s.ball_pos))
        if math.hypot(pos[0] - bx, pos[1] - by) < HIT_RADIUS_PX:
            return "ball"
        rx, ry = field_to_screen(*frames.sim_to_fw_pos(s.robot_pos))
        if math.hypot(pos[0] - rx, pos[1] - ry) < ROBOT_RADIUS * 100 * SCALE + 5:
            return "robot"
        return None
