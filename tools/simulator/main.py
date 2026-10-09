#!/usr/bin/env python3
"""Bucky Field Simulator — dual-panel visualization for RoboCup Junior robot."""

import os
import sys
import pygame

from config.cpp_parser import parse_all, get_mtimes, has_changed
from simulation.world_state import WorldState
from simulation.sensor_model import simulate_sensors, SensorReadings
from simulation.firmware_model import FirmwareModel, PerceivedState
from rendering.colors import BG_RIGHT, BG_LEFT, DIVIDER, HUD_BG, TEXT_COLOR, TEXT_DIM
from rendering.panel_layout import WINDOW_W, WINDOW_H, PANEL_W, HUD_H
from rendering.field_renderer import draw_field
from rendering.perception_renderer import draw_perception
from interaction.drag_controller import DragController

FPS = 60
HOT_RELOAD_INTERVAL = 60  # frames between mtime checks

# Resolve source root relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Bucky Field Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 13)

    # Parse C++ source
    config = parse_all(SRC_ROOT)
    mtimes = get_mtimes(SRC_ROOT)

    world = WorldState()
    firmware = FirmwareModel(config)
    drag = DragController()

    frame_count = 0
    show_overlays = False
    running = True

    while running:
        dt = clock.tick(FPS) / 1000.0

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    world.reset()
                    firmware.reset()
                elif event.key == pygame.K_SPACE:
                    world.paused = not world.paused
                elif event.key == pygame.K_s:
                    show_overlays = not show_overlays
            drag.handle_event(event, world)

        # Hot-reload C++ config
        frame_count += 1
        if frame_count % HOT_RELOAD_INTERVAL == 0:
            if has_changed(SRC_ROOT, mtimes):
                config = parse_all(SRC_ROOT)
                mtimes = get_mtimes(SRC_ROOT)
                firmware = FirmwareModel(config)

        # Simulate sensors from ground truth
        readings = simulate_sensors(world, config)

        # Run firmware model to get perceived state
        perceived = firmware.update(readings, config, dt)

        # Draw
        screen.fill(BG_LEFT, (0, 0, PANEL_W, WINDOW_H - HUD_H))
        screen.fill(BG_RIGHT, (PANEL_W, 0, PANEL_W, WINDOW_H - HUD_H))
        pygame.draw.line(screen, DIVIDER, (PANEL_W, 0), (PANEL_W, WINDOW_H - HUD_H), 2)

        draw_field(screen, world, config, font, show_overlays)
        draw_perception(screen, world, readings, perceived, config, font)

        # HUD bar
        screen.fill(HUD_BG, (0, WINDOW_H - HUD_H, WINDOW_W, HUD_H))
        fps_text = font.render(f"FPS: {clock.get_fps():.0f}", True, TEXT_DIM)
        screen.blit(fps_text, (WINDOW_W - 80, WINDOW_H - HUD_H + 8))

        cfg_text = font.render(
            f"IR: {config.ir.total_sensor_count}sensors {config.ir.mode}  "
            f"Compass: Kp={config.compass.kp} Kd={config.compass.kd}  "
            f"Sonar: {config.sonar.count}ch  "
            f"[S]overlays [R]eset [Space]pause",
            True, TEXT_DIM,
        )
        screen.blit(cfg_text, (10, WINDOW_H - HUD_H + 8))

        # Panel titles
        title_l = font.render("ROBOT PERCEPTION", True, TEXT_COLOR)
        title_r = font.render("ACTUAL FIELD", True, TEXT_COLOR)
        screen.blit(title_l, (PANEL_W // 2 - title_l.get_width() // 2, 8))
        screen.blit(title_r, (PANEL_W + PANEL_W // 2 - title_r.get_width() // 2, 8))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
