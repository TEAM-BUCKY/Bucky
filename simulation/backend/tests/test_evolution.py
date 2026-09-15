"""Black-box optimizers converge on a known function; genomes save to loadable artifacts."""
import numpy as np
import pytest

from bucky.algos.evolution import (
    ControllerGenome,
    MLPGenome,
    evolve_cem,
    evolve_ga,
)


def _sphere(x):
    return -float(np.sum(x ** 2))   # maximized at the origin


@pytest.mark.parametrize("optimizer", [evolve_cem, evolve_ga])
def test_optimizer_converges_toward_optimum(optimizer):
    best_vec, best_fit = optimizer(_sphere, dim=6, x0=np.full(6, 2.0), sigma=1.0,
                                   generations=40, popsize=32, seed=0)
    # Starting fitness at x0 is -24; a working optimizer gets much closer to 0.
    assert best_fit > -1.0
    assert np.linalg.norm(best_vec) < 1.0


def test_mlp_genome_dim_and_saves_loadable_npz(tmp_path):
    from bucky.selfplay import load_numpy_opponent

    genome = MLPGenome([39, 16, 4])
    assert genome.dim == 39 * 16 + 16 + 16 * 4 + 4
    vec = np.random.default_rng(0).standard_normal(genome.dim)
    path = str(tmp_path / "policy.npz")
    genome.save(vec, path)
    opp = load_numpy_opponent(path)              # must load via the existing numpy-opponent adapter
    action, _ = opp.predict(np.zeros(39, np.float32))
    assert action.shape == (4,)


def test_controller_genome_saves_loadable_json(tmp_path):
    from bucky.policies import load_policy

    genome = ControllerGenome()
    vec = genome.x0()
    assert genome.dim == len(vec)
    path = str(tmp_path / "tuned.controller.json")
    genome.save(vec, path)
    policy = load_policy(path)                    # must load via the classical-controller path
    action, _ = policy.predict(np.zeros(39, np.float32))
    assert action.shape == (4,)
