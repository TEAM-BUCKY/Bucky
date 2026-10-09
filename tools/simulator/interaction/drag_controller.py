"""Handle mouse drag for ball/robot and keyboard input."""

import pygame
from simulation.world_state import WorldState, ROBOT_RADIUS, BALL_RADIUS
from rendering.panel_layout import (
    world_to_screen, screen_to_world, is_in_right_panel, FIELD_SCALE,
)

HIT_RADIUS_PX = 15  # generous click target


class DragController:
    def __init__(self):
        self._dragging = None  # "ball" or "robot" or None

    def handle_event(self, event: pygame.event.Event, world: WorldState):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # left click
                self._try_start_drag(event.pos, world)
            elif event.button == 3:  # right click — place ball
                if is_in_right_panel(*event.pos):
                    wx, wy = screen_to_world(*event.pos)
                    world.ball_x = wx
                    world.ball_y = wy
            elif event.button == 2:  # middle click — place robot
                if is_in_right_panel(*event.pos):
                    wx, wy = screen_to_world(*event.pos)
                    world.robot_x = wx
                    world.robot_y = wy

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self._dragging = None

        elif event.type == pygame.MOUSEMOTION:
            if self._dragging and is_in_right_panel(*event.pos):
                wx, wy = screen_to_world(*event.pos)
                if self._dragging == "ball":
                    world.ball_x = wx
                    world.ball_y = wy
                elif self._dragging == "robot":
                    world.robot_x = wx
                    world.robot_y = wy

        elif event.type == pygame.MOUSEWHEEL:
            # Scroll wheel rotates robot heading
            mx, my = pygame.mouse.get_pos()
            rx, ry = world_to_screen(world.robot_x, world.robot_y)
            dist = ((mx - rx) ** 2 + (my - ry) ** 2) ** 0.5
            robot_r_px = ROBOT_RADIUS * FIELD_SCALE
            if dist < robot_r_px * 3:  # generous range for scroll
                world.robot_heading = (world.robot_heading + event.y * 5) % 360

    def _try_start_drag(self, pos: tuple[int, int], world: WorldState):
        if not is_in_right_panel(*pos):
            return

        sx, sy = pos

        # Check ball first (smaller target, prioritize)
        bx, by = world_to_screen(world.ball_x, world.ball_y)
        if (sx - bx) ** 2 + (sy - by) ** 2 < HIT_RADIUS_PX ** 2:
            self._dragging = "ball"
            return

        # Check robot
        rx, ry = world_to_screen(world.robot_x, world.robot_y)
        robot_r_px = ROBOT_RADIUS * FIELD_SCALE
        if (sx - rx) ** 2 + (sy - ry) ** 2 < (robot_r_px + 5) ** 2:
            self._dragging = "robot"
            return
