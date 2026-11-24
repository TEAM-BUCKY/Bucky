"""DSAC, Distributional Soft Actor-Critic (Ma et al. 2020; PWS §9.2.7).

DSAC keeps SAC's structure: a stochastic actor, a soft critic and an entropy bonus. The
difference is the critic. Instead of one expected value Q(s, a), it predicts the full
distribution of future reward, as quantiles θ_1 … θ_N (quantile regression, as in QR-DQN).

Because the actor sees a distribution, it can be **risk-sensitive**. That is the reason the
PWS gives for wanting DSAC: in a match, taking a risk is a deliberate choice. The actor
maximises a risk measure ρ over the critic's quantiles instead of their mean:

* ``risk="neutral"``: ρ = mean of all quantiles, the ordinary expected-value SAC objective.
* ``risk="cvar"``: ρ = mean of the lowest ``risk_alpha`` fraction (CVaR_α). This is
  risk-averse: it plays for the worst case, e.g. the safe route via its own goal.
* ``risk="optimistic"``: ρ = mean of the highest ``risk_alpha`` fraction. This is risk-seeking:
  it goes for the high-upside, risky play.

The implementation builds on sb3-contrib's TQC, which already has a quantile critic, the target
networks and the entropy tuning. Two things change:

1. Truncation is off by default (``top_quantiles_to_drop_per_net=0``). TQC drops the top
   quantiles to fight overestimation, which plain DSAC does not do.
2. ``train()`` is TQC's, except that the actor loss uses the risk measure above. The critic
   update is unchanged quantile-Huber regression.
"""
from __future__ import annotations

import numpy as np
import torch as th
from sb3_contrib import TQC
from sb3_contrib.common.utils import quantile_huber_loss
from stable_baselines3.common.utils import polyak_update

RISK_MEASURES = ("neutral", "cvar", "optimistic")


def risk_measure(quantiles: th.Tensor, risk: str = "neutral", alpha: float = 0.25) -> th.Tensor:
    """Reduce critic quantiles (B, n_critics, n_quantiles) to one value per sample (B, 1)."""
    flat = quantiles.reshape(quantiles.shape[0], -1)
    if risk == "neutral":
        return flat.mean(dim=1, keepdim=True)
    if risk not in RISK_MEASURES:
        raise ValueError(f"unknown risk measure {risk!r}; choose from {RISK_MEASURES}")
    k = max(1, int(round(alpha * flat.shape[1])))
    ordered, _ = th.sort(flat, dim=1)
    part = ordered[:, :k] if risk == "cvar" else ordered[:, -k:]
    return part.mean(dim=1, keepdim=True)


class DSAC(TQC):
    def __init__(self, *args, risk: str = "neutral", risk_alpha: float = 0.25,
                 top_quantiles_to_drop_per_net: int = 0, **kwargs) -> None:
        if risk not in RISK_MEASURES:
            raise ValueError(f"unknown risk measure {risk!r}; choose from {RISK_MEASURES}")
        if not 0.0 < float(risk_alpha) <= 1.0:
            raise ValueError("risk_alpha must be in (0, 1]")
        self.risk = risk
        self.risk_alpha = float(risk_alpha)
        super().__init__(*args, top_quantiles_to_drop_per_net=top_quantiles_to_drop_per_net, **kwargs)

    def train(self, gradient_steps: int, batch_size: int = 64) -> None:
        self.policy.set_training_mode(True)
        optimizers = [self.actor.optimizer, self.critic.optimizer]
        if self.ent_coef_optimizer is not None:
            optimizers += [self.ent_coef_optimizer]
        self._update_learning_rate(optimizers)

        ent_coef_losses, ent_coefs, actor_losses, critic_losses = [], [], [], []
        for gradient_step in range(gradient_steps):
            data = self.replay_buffer.sample(batch_size, env=self._vec_normalize_env)  # type: ignore[union-attr]
            discounts = data.discounts if data.discounts is not None else self.gamma
            if self.use_sde:
                self.actor.reset_noise()

            actions_pi, log_prob = self.actor.action_log_prob(data.observations)
            log_prob = log_prob.reshape(-1, 1)

            # ── entropy temperature (unchanged SAC/TQC) ──
            ent_coef_loss = None
            if self.ent_coef_optimizer is not None and self.log_ent_coef is not None:
                ent_coef = th.exp(self.log_ent_coef.detach())
                ent_coef_loss = -(self.log_ent_coef * (log_prob + self.target_entropy).detach()).mean()
                ent_coef_losses.append(ent_coef_loss.item())
            else:
                ent_coef = self.ent_coef_tensor
            ent_coefs.append(ent_coef.item())
            if ent_coef_loss is not None and self.ent_coef_optimizer is not None:
                self.ent_coef_optimizer.zero_grad()
                ent_coef_loss.backward()
                self.ent_coef_optimizer.step()

            # ── distributional critic: quantile regression on the soft TD target ──
            with th.no_grad():
                next_actions, next_log_prob = self.actor.action_log_prob(data.next_observations)
                next_q = self.critic_target(data.next_observations, next_actions)
                n_target = self.critic.quantiles_total - self.top_quantiles_to_drop_per_net * self.critic.n_critics
                next_q, _ = th.sort(next_q.reshape(batch_size, -1))
                next_q = next_q[:, :n_target]
                target = next_q - ent_coef * next_log_prob.reshape(-1, 1)
                target = data.rewards + (1 - data.dones) * discounts * target
                target.unsqueeze_(dim=1)

            current = self.critic(data.observations, data.actions)
            critic_loss = quantile_huber_loss(current, target, sum_over_quantiles=False)
            critic_losses.append(critic_loss.item())
            self.critic.optimizer.zero_grad()
            critic_loss.backward()
            self.critic.optimizer.step()

            # ── risk-sensitive actor (the DSAC change) ──
            rho = risk_measure(self.critic(data.observations, actions_pi), self.risk, self.risk_alpha)
            actor_loss = (ent_coef * log_prob - rho).mean()
            actor_losses.append(actor_loss.item())
            self.actor.optimizer.zero_grad()
            actor_loss.backward()
            self.actor.optimizer.step()

            if gradient_step % self.target_update_interval == 0:
                polyak_update(self.critic.parameters(), self.critic_target.parameters(), self.tau)
                polyak_update(self.batch_norm_stats, self.batch_norm_stats_target, 1.0)

        self._n_updates += gradient_steps
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/ent_coef", np.mean(ent_coefs))
        self.logger.record("train/actor_loss", np.mean(actor_losses))
        self.logger.record("train/critic_loss", np.mean(critic_losses))
        if ent_coef_losses:
            self.logger.record("train/ent_coef_loss", np.mean(ent_coef_losses))
