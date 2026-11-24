"""PWS §9 scripted algorithms: obs decoding, FSM / APF / behaviour tree controllers."""
import numpy as np
import pytest

from bucky.controllers import SCRIPTED_CONTROLLERS
from bucky.controllers import skills
from bucky.controllers.behaviour_tree import BehaviourTreeController
from bucky.controllers.fsm import FSMController, Inputs, State, next_state
from bucky.controllers.perception import decode_obs
from bucky.obs import build_observation
from bucky.physics.backend import PhysicsState
from bucky.policies.loader import load_policy
from bucky.selfplay import sonar_ranges


def _state(robot=(-0.3, 0.1), heading=0.4, ball=(0.2, -0.1), ball_vel=(0.3, 0.0)):
    return PhysicsState(robot_pos=np.array(robot, float), robot_vel=np.array([0.2, -0.1]),
                        robot_heading=heading, robot_omega=0.0,
                        ball_pos=np.array(ball, float), ball_vel=np.array(ball_vel, float))


def _obs(state, opponent=None):
    o = build_observation(state, opponent_pos=opponent)
    if opponent is None:
        return o
    return np.concatenate([o, sonar_ranges(state.robot_pos, state.robot_heading, opponent)])


def test_decode_obs_inverts_build_observation():
    rng = np.random.default_rng(1)
    for _ in range(200):
        s = _state(robot=rng.uniform([-0.8, -0.5], [0.8, 0.5]), heading=rng.uniform(-3.1, 3.1),
                   ball=rng.uniform([-0.8, -0.5], [0.8, 0.5]), ball_vel=rng.uniform(-1, 1, 2))
        v = decode_obs(_obs(s))
        assert np.allclose(v.robot_pos, s.robot_pos, atol=1e-5)
        assert np.allclose(v.ball_pos, s.ball_pos, atol=1e-5)
        assert np.allclose(v.ball_vel, s.ball_vel, atol=1e-4)
        assert abs(np.angle(np.exp(1j * (v.heading - s.robot_heading)))) < 1e-5
        assert v.opponent_pos is None                      # single-agent obs has no sonar


def test_sonar_opponent_estimate_is_in_the_right_area():
    s = _state(robot=(0.0, 0.0), heading=0.0)
    v = decode_obs(_obs(s, opponent=np.array([0.5, 0.0])))    # straight ahead
    assert v.opponent_pos is not None
    assert np.linalg.norm(v.opponent_pos - [0.5, 0.0]) < 0.15


def test_bisector_approach_point_settles_behind_ball():
    # §5: robot already at C (10 cm behind the ball, on the goal line) → A = C: it stays there.
    s = _state(robot=(0.3 - skills.BEHIND_DIST, 0.0), heading=0.0, ball=(0.3, 0.0))
    A = skills.approach_point(decode_obs(_obs(s)))
    assert np.allclose(A, [0.3 - skills.BEHIND_DIST, 0.0], atol=1e-3)
    # Robot farther behind on the line → A lies between robot and ball at (|BC| + |BO|)/2.
    s = _state(robot=(0.0, 0.0), heading=0.0, ball=(0.3, 0.0))
    A = skills.approach_point(decode_obs(_obs(s)))
    assert np.allclose(A, [0.3 - (skills.BEHIND_DIST + 0.3) / 2, 0.0], atol=1e-3)
    # The robot in front of the ball → A swings to the side (does not drive through the ball).
    s = _state(robot=(0.6, 0.0), heading=np.pi, ball=(0.3, 0.0))
    A = skills.approach_point(decode_obs(_obs(s)))
    assert abs(A[1]) > 0.1


def test_fsm_transition_table():
    s_behind = _state(robot=(0.0, 0.0), heading=0.0, ball=(0.15, 0.0), ball_vel=(0, 0))
    i = Inputs.read(decode_obs(_obs(s_behind)))
    assert next_state(State.APPROACH, i) == State.ATTACK
    s_far = _state(robot=(-0.6, 0.0), heading=0.0, ball=(0.4, 0.3), ball_vel=(0, 0))
    assert next_state(State.ATTACK, Inputs.read(decode_obs(_obs(s_far)))) == State.APPROACH
    s_out = _state(ball=(0.5, 0.75), ball_vel=(0, 0))
    assert next_state(State.ATTACK, Inputs.read(decode_obs(_obs(s_out)))) == State.RECOVER
    # Ball in our half, opponent right next to it → DEFEND.
    s_threat = _state(robot=(0.3, 0.0), heading=np.pi, ball=(-0.4, 0.0), ball_vel=(0, 0))
    i = Inputs.read(decode_obs(_obs(s_threat, opponent=np.array([-0.55, 0.0]))))
    assert next_state(State.APPROACH, i) == State.DEFEND


@pytest.mark.parametrize("name", sorted(SCRIPTED_CONTROLLERS))
def test_controllers_give_valid_actions(name):
    pol = load_policy(name)
    rng = np.random.default_rng(0)
    for _ in range(100):
        s = _state(robot=rng.uniform([-0.8, -0.5], [0.8, 0.5]), heading=rng.uniform(-3, 3),
                   ball=rng.uniform([-0.85, -0.55], [0.85, 0.55]))
        for obs in (_obs(s), _obs(s, opponent=rng.uniform([-0.8, -0.5], [0.8, 0.5]))):
            act, _ = pol.predict(obs)
            assert act.shape == (4,) and np.all(np.isfinite(act)) and np.all(np.abs(act) <= 1.0)


def test_attack_kicks_when_lined_up():
    s = _state(robot=(0.3, 0.0), heading=0.0, ball=(0.43, 0.0), ball_vel=(0, 0))
    fsm = FSMController()
    fsm.state = State.ATTACK
    act, _ = fsm.predict(_obs(s))
    assert act[3] == 1.0


def test_behaviour_tree_trace():
    bt = BehaviourTreeController()
    bt.predict(_obs(_state(robot=(-0.6, 0.0), ball=(0.4, 0.3))))
    assert bt.last_trace == ["approach"]
    bt.predict(_obs(_state(robot=(0.3, 0.0), heading=0.0, ball=(0.43, 0.0), ball_vel=(0, 0))))
    assert bt.last_trace == ["shoot/dribble"]


@pytest.mark.parametrize("name", sorted(SCRIPTED_CONTROLLERS))
def test_scripted_controller_scores_against_classical(name):
    from bucky.match import MatchEngine
    eng = MatchEngine(load_policy(name), load_policy("classical"), seed=0,
                      domain_rand=False, match_mode=True)
    frame = None
    for _ in range(6000):                               # 2 min of play
        frame = eng.tick()
    assert frame["score"]["a"] >= 1
