"""The pygame viewer runs headless: frames render, drags move things, programs switch."""
from __future__ import annotations

import os

import pytest

pygame = pytest.importorskip("pygame")


def test_viewer_headless(sim, monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    from bucky.firmware import frames
    from bucky.firmware.viewer.app import run
    from bucky.firmware.viewer.layout import field_to_screen

    ball0 = (0.3, 0.1)
    bx, by = field_to_screen(*frames.sim_to_fw_pos(ball0))
    tx, ty = field_to_screen(*frames.sim_to_fw_pos((0.0, -0.3)))
    E = pygame.event.Event
    events = {
        5: [E(pygame.MOUSEBUTTONDOWN, button=1, pos=(int(bx), int(by)))],
        6: [E(pygame.MOUSEMOTION, pos=(int(tx), int(ty)), rel=(0, 0), buttons=(1, 0, 0))],
        7: [E(pygame.MOUSEBUTTONUP, button=1, pos=(int(tx), int(ty)))],
        10: [E(pygame.KEYDOWN, key=pygame.K_p, mod=0, unicode="p")],
    }
    v = run("testIRPositioning", frames=90, events=events, ball=ball0)
    assert v.world.program != "testIRPositioning"        # P switched program
    assert v.banner is None
    ball = v.world.physics.state_a().ball_pos              # the drag moved the ball
    assert abs(ball[0]) < 0.03 and abs(ball[1] + 0.3) < 0.03
    assert os.environ["SDL_VIDEODRIVER"] == "dummy"
