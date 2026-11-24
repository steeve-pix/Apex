import torch
from torch import nn


def update_ppo(driver, buffer):
    observations, actions, old_log_probs, returns, advantages = (
        buffer.training_batch()
    )

    advantages = ((advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8))

    metrics = []
    minibatch_size = 64
    epochs = 4
    clip_range = 0.2

    for _ in range(epochs):
        order = torch.randperm(len(buffer))

        for start in range(0, len(buffer), minibatch_size):
            indices = order[start:start + minibatch_size]
            log_probs, values, entropy = driver.evaluate_actions(
                observations[indices],
                actions[indices],
            )

            ratios = (log_probs - old_log_probs[indices]).exp()
            direct = ratios * advantages[indices]
            clipped = ratios.clamp(
                1.0 - clip_range, 1.0 + clip_range
            ) * advantages[indices]

            policy_loss = -torch.minimum(direct, clipped).mean()
            value_loss = 0.5 * (
                    returns[indices] - values
            ).square().mean()
            entropy_bonus = entropy.mean()

            loss = (policy_loss + value_loss - 0.01 * entropy_bonus)

            if not torch.isfinite(loss):
                raise RuntimeError("PPO loss became non-finite")

            driver.optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(driver.parameters(), 0.5)
            driver.optimizer.step()

            metrics.append((
                policy_loss.item(),
                value_loss.item(),
                entropy_bonus.item(),
            ))

    return {
        name: sum(row[index] for row in metrics) / len(metrics)
        for index, name in enumerate(
            ("policy_loss", "value_loss", "entropy")
        )
    }
