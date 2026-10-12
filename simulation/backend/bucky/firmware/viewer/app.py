"""The viewer's main loop: real-time pacing, keyboard/mouse, panels.

Keys: Space pause · . single step (paused) · R reboot · 1 / 2 press a button · P next program
      + / - simulation speed · O sensor overlays · C clear serial · Esc quit
Mouse (field panel): drag ball / robot · right-click drops the ball · scroll rotates the robot
"""
from __future__ import annotations

from collections import deque

import pygame

from bucky.firmware.programs import list_programs
from bucky.firmware.viewer import colors as C
from bucky.firmware.viewer.drag import DragController
from bucky.firmware.viewer.field import draw_field
from bucky.firmware.viewer.layout import HUD_H, PANEL_W, SERIAL_H, VIEW_H, WINDOW_H, WINDOW_W
from bucky.firmware.viewer.perception import draw_perception
from bucky.firmware.viewer.probes import Perceived
from bucky.firmware.world import FirmwareError, FirmwareWorld

FPS = 60
MAX_STEPS_PER_FRAME = 20
SPEEDS = (0.1, 0.25, 0.5, 1.0, 2.0, 4.0)


class Viewer:
    def __init__(self, program: str = "main_loop", *, robot=(-0.4, 0.0), heading=0.0,
                 ball=(0.3, 0.1), seed: int = 0) -> None:
        self.world = FirmwareWorld(program, seed=seed, warmup_s=0.0)
        self.start = (robot, heading, ball)
        self.programs = list_programs()
        self.paused = False
        self.overlays = True
        self.speed_i = SPEEDS.index(1.0)
        self.serial: deque[str] = deque(maxlen=400)
        self.perceived = Perceived()
        self.banner: str | None = None
        self.drag = DragController()
        self._debt = 0.0
        self.reboot(program, initial=True)

    # ── control ───────────────────────────────────────────────────────────────────────────
    def reboot(self, program: str | None = None, initial: bool = False) -> None:
        """Boot ``program`` (default: the current one) with the robot and ball where they are
        now (or at the start positions, the first time)."""
        robot, heading, ball = self.start
        if not initial:
            s = self.world.physics.state_a()
            robot, heading, ball = s.robot_pos, s.robot_heading, s.ball_pos
        self.world.place(robot, heading, ball)
        self.world.boot(program)
        self.serial.clear()
        self.perceived = Perceived()
        self.banner = None

    def step(self) -> None:
        if self.banner is not None:
            return
        try:
            self.world.step()
        except (FirmwareError, self.world.fw.SimError) as e:
            self.banner = str(e)
            return
        for line in self.world.serial.new_lines():
            self.serial.append(line)
            self.perceived.feed(line)

    def handle(self, event) -> bool:
        """Returns False to quit."""
        if event.type == pygame.QUIT:
            return False
        if event.type == pygame.KEYDOWN:
            k = event.key
            if k == pygame.K_ESCAPE:
                return False
            if k == pygame.K_SPACE:
                self.paused = not self.paused
            elif k == pygame.K_PERIOD and self.paused:
                self.step()
            elif k == pygame.K_r:
                self.reboot(self.world.program)
            elif k == pygame.K_p:
                cur = self.world.program
                i = self.programs.index(cur) if cur in self.programs else -1
                self.reboot(self.programs[(i + 1) % len(self.programs)])
            elif k in (pygame.K_1, pygame.K_2):
                self.world.hw.press_button(1 if k == pygame.K_1 else 2)
            elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.speed_i = min(self.speed_i + 1, len(SPEEDS) - 1)
            elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.speed_i = max(self.speed_i - 1, 0)
            elif k == pygame.K_o:
                self.overlays = not self.overlays
            elif k == pygame.K_c:
                self.serial.clear()
        self.drag.handle_event(event, self.world)
        return True

    def advance(self, wall_dt: float) -> None:
        if self.paused:
            return
        self._debt += wall_dt * SPEEDS[self.speed_i]
        n = 0
        while self._debt >= self.world.dt and n < MAX_STEPS_PER_FRAME:
            self.step()
            self._debt -= self.world.dt
            n += 1
        if n == MAX_STEPS_PER_FRAME:
            self._debt = 0.0     # can't keep up: slow down rather than spiral

    # ── drawing ───────────────────────────────────────────────────────────────────────────
    def draw(self, screen, font) -> None:
        screen.fill(C.BG_LEFT, (0, 0, PANEL_W, VIEW_H))
        screen.fill(C.BG_RIGHT, (PANEL_W, 0, PANEL_W, VIEW_H))
        pygame.draw.line(screen, C.DIVIDER, (PANEL_W, 0), (PANEL_W, VIEW_H), 2)
        draw_field(screen, self.world, self.overlays)
        draw_perception(screen, font, self.world, self.perceived)
        for text, x in (("ROBOT PERCEPTION (firmware)", PANEL_W // 2),
                        ("ACTUAL FIELD", PANEL_W + PANEL_W // 2)):
            t = font.render(text, True, C.TEXT_COLOR)
            screen.blit(t, (x - t.get_width() // 2, 8))
        prog = font.render(f"program: {self.world.program}", True, C.TEXT_COLOR)
        screen.blit(prog, (12, 22))

        # serial pane
        screen.fill(C.HUD_BG, (0, VIEW_H, WINDOW_W, SERIAL_H))
        pygame.draw.line(screen, C.DIVIDER, (0, VIEW_H), (WINDOW_W, VIEW_H), 1)
        rows = (SERIAL_H - 8) // 15
        for i, line in enumerate(list(self.serial)[-rows:]):
            screen.blit(font.render(line.expandtabs(6)[:180], True, C.TEXT_COLOR),
                        (10, VIEW_H + 4 + i * 15))

        # HUD
        screen.fill(C.HUD_BG, (0, WINDOW_H - HUD_H, WINDOW_W, HUD_H))
        sim = self.world.fw.sim
        state = "PAUSED" if self.paused else f"x{SPEEDS[self.speed_i]:g}"
        hud = (f"t={sim.now_us() / 1e6:7.2f}s  {state}  firmware: {sim.status()}   "
               "[Space] pause [.] step [R] reboot [P] program [1/2] buttons [+/-] speed "
               "[O] overlays [C] clear")
        screen.blit(font.render(hud, True, C.TEXT_DIM), (10, WINDOW_H - HUD_H + 8))
        if self.banner:
            pygame.draw.rect(screen, C.BANNER, (0, VIEW_H // 2 - 20, WINDOW_W, 40))
            screen.blit(font.render(self.banner[:200], True, (255, 255, 255)),
                        (10, VIEW_H // 2 - 8))

    def close(self) -> None:
        self.world.close()


def run(program: str = "main_loop", *, frames: int | None = None, events=None, **kw) -> Viewer:
    """Open the window and run until closed (or for ``frames`` frames, feeding ``events`` —
    a {frame: [pygame.event.Event, ...]} map — for scripted/headless runs)."""
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Bucky firmware simulator")
    font = pygame.font.SysFont("monospace", 13)
    clock = pygame.time.Clock()
    viewer = Viewer(program, **kw)
    frame = 0
    try:
        running = True
        while running and (frames is None or frame < frames):
            wall_dt = clock.tick(FPS) / 1000.0 if frames is None else 1.0 / FPS
            queued = list((events or {}).get(frame, []))
            for event in pygame.event.get() + queued:
                running = viewer.handle(event) and running
            viewer.advance(min(wall_dt, 0.1))
            viewer.draw(screen, font)
            pygame.display.flip()
            frame += 1
    finally:
        viewer.close()
        pygame.quit()
    return viewer
