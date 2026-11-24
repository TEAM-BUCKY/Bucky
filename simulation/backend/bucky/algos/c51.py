"""C51, Categorical DQN (Bellemare, Dabney & Munos 2017; PWS §9.2.5).

Normal DQN learns one number per action: the *expected* future reward Q(s, a). C51 learns the
whole *distribution* Z(s, a) of future reward instead, as probabilities over 51 fixed values
("atoms") z_0 … z_50, spaced evenly between ``v_min`` and ``v_max``. The PWS example: action A
always gives 10 points, while action B gives 0 or 20. Both have Q = 10, but C51 sees the
different spread, i.e. the risk.

* **Acting:** Q(s, a) = Σ_i z_i · p_i(s, a), then the greedy argmax (ε-greedy while training,
  exactly like DQN).
* **Learning:** the target distribution r + γ·Z(s', a*) shifts every atom to r + γ·z_i. Those
  shifted atoms no longer line up with the fixed grid, so their probability mass is *projected*
  back onto the two nearest atoms (:func:`categorical_projection`). The loss is the
  cross-entropy between the projected target and the predicted distribution.

The implementation is SB3's DQN (replay buffer, target network, ε-schedule, save/load) with a
categorical Q-network and a categorical ``train()``.
"""
from __future__ import annotations

from typing import Any, ClassVar

import numpy as np
import torch as th
from stable_baselines3 import DQN
from stable_baselines3.common.policies import BasePolicy
from stable_baselines3.common.torch_layers import create_mlp
from stable_baselines3.dqn.policies import DQNPolicy, QNetwork
from torch import nn
from torch.nn import functional as F

DEFAULT_ATOMS = 51
# Return range covered by the atoms. With the PWS reward an episode is worth roughly
# −(10 + 5 + …) … +(10 + 5 + 4 + 2) per goal, so ±20 spans the useful range. Override it for
# other reward scales: returns outside the range are clipped onto the edge atoms.
DEFAULT_V_MIN = -20.0
DEFAULT_V_MAX = 20.0


def categorical_projection(next_probs: th.Tensor, rewards: th.Tensor, dones: th.Tensor,
                           gamma: th.Tensor | float, support: th.Tensor) -> th.Tensor:
    """Project the distribution r + γ·(1 − done)·z onto the fixed ``support`` (Bellemare 2017, Alg. 1).

    ``next_probs`` (B, N) are the target probabilities of the greedy next action.
    ``rewards`` and ``dones`` have shape (B, 1). Returns (B, N) probabilities, and each row
    sums to 1.
    """
    n_atoms = support.shape[0]
    v_min, v_max = float(support[0]), float(support[-1])
    delta_z = (v_max - v_min) / (n_atoms - 1)

    tz = (rewards + (1.0 - dones) * gamma * support.unsqueeze(0)).clamp(v_min, v_max)   # (B, N)
    b = (tz - v_min) / delta_z                     # fractional atom index
    lower, upper = b.floor().long(), b.ceil().long()
    # When b lands exactly on an atom, lower == upper and the mass would vanish; nudge it.
    lower[(upper > 0) & (lower == upper)] -= 1
    upper[(lower < n_atoms - 1) & (lower == upper)] += 1

    proj = th.zeros_like(next_probs)
    proj.scatter_add_(1, lower, next_probs * (upper.float() - b))
    proj.scatter_add_(1, upper, next_probs * (b - lower.float()))
    return proj


class CategoricalQNetwork(QNetwork):
    """Outputs per-action atom logits. ``forward`` returns the expected Q-values, so all of
    SB3's DQN machinery (ε-greedy predict, target updates) works unchanged."""

    def __init__(self, *args, n_atoms: int = DEFAULT_ATOMS, v_min: float = DEFAULT_V_MIN,
                 v_max: float = DEFAULT_V_MAX, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.n_atoms, self.v_min, self.v_max = int(n_atoms), float(v_min), float(v_max)
        n_actions = int(self.action_space.n)
        self.q_net = nn.Sequential(*create_mlp(self.features_dim, n_actions * self.n_atoms,
                                               self.net_arch, self.activation_fn))
        self.register_buffer("support", th.linspace(self.v_min, self.v_max, self.n_atoms))

    def logits(self, obs) -> th.Tensor:
        out = self.q_net(self.extract_features(obs, self.features_extractor))
        return out.view(-1, int(self.action_space.n), self.n_atoms)

    def probs(self, obs) -> th.Tensor:
        return F.softmax(self.logits(obs), dim=-1)

    def forward(self, obs) -> th.Tensor:
        return (self.probs(obs) * self.support).sum(dim=-1)

    def _get_constructor_parameters(self) -> dict[str, Any]:
        data = super()._get_constructor_parameters()
        data.update(n_atoms=self.n_atoms, v_min=self.v_min, v_max=self.v_max)
        return data


class C51Policy(DQNPolicy):
    def __init__(self, *args, n_atoms: int = DEFAULT_ATOMS, v_min: float = DEFAULT_V_MIN,
                 v_max: float = DEFAULT_V_MAX, **kwargs) -> None:
        self.n_atoms, self.v_min, self.v_max = int(n_atoms), float(v_min), float(v_max)
        super().__init__(*args, **kwargs)

    def make_q_net(self) -> CategoricalQNetwork:
        net_args = self._update_features_extractor(self.net_args, features_extractor=None)
        return CategoricalQNetwork(**net_args, n_atoms=self.n_atoms, v_min=self.v_min,
                                   v_max=self.v_max).to(self.device)

    def _get_constructor_parameters(self) -> dict[str, Any]:
        data = super()._get_constructor_parameters()
        data.update(n_atoms=self.n_atoms, v_min=self.v_min, v_max=self.v_max)
        return data


class C51(DQN):
    policy_aliases: ClassVar[dict[str, type[BasePolicy]]] = {"MlpPolicy": C51Policy}
    q_net: CategoricalQNetwork
    q_net_target: CategoricalQNetwork

    def train(self, gradient_steps: int, batch_size: int = 100) -> None:
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)

        losses = []
        for _ in range(gradient_steps):
            data = self.replay_buffer.sample(batch_size, env=self._vec_normalize_env)  # type: ignore[union-attr]
            discounts = data.discounts if data.discounts is not None else self.gamma
            support = self.q_net.support

            with th.no_grad():
                next_probs = self.q_net_target.probs(data.next_observations)          # (B, A, N)
                next_q = (next_probs * support).sum(-1)                               # (B, A)
                best = next_q.argmax(dim=1)
                next_best = next_probs[th.arange(batch_size, device=best.device), best]  # (B, N)
                target = categorical_projection(next_best, data.rewards, data.dones,
                                                discounts, support)

            logits = self.q_net.logits(data.observations)                             # (B, A, N)
            idx = data.actions.long().view(-1)
            log_p = F.log_softmax(logits[th.arange(batch_size, device=idx.device), idx], dim=-1)
            loss = -(target * log_p).sum(dim=-1).mean()                               # cross-entropy
            losses.append(loss.item())

            self.policy.optimizer.zero_grad()
            loss.backward()
            th.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
            self.policy.optimizer.step()

        self._n_updates += gradient_steps
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/loss", np.mean(losses))
