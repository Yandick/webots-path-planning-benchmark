"""Minimal PPO for small discrete policies, implemented in NumPy.

The observation space here is ~20-D and the action space has 3 discrete
actions, so a hand-rolled clipped-surrogate PPO with Adam keeps the project
dependency-light and the training loop fully reproducible without a GPU.
"""
import math

import numpy as np


def _orthogonal(fan_in, fan_out, rng, scale=math.sqrt(2.)):
    a = rng.normal(size=(fan_in, fan_out))
    u, _, vt = np.linalg.svd(a, full_matrices=False)
    q = u if u.shape == a.shape else vt
    return q*scale


class PPOPolicy:
    def __init__(self, obs_dim, act_dim=3, hidden=64, seed=0):
        rng = np.random.default_rng(seed)
        self.obs_dim, self.act_dim, self.hidden = obs_dim, act_dim, hidden
        self.p = {"W1": _orthogonal(obs_dim, hidden, rng), "b1": np.zeros(hidden),
                  "W2": _orthogonal(hidden, hidden, rng), "b2": np.zeros(hidden),
                  "W3": _orthogonal(hidden, act_dim, rng, .01), "b3": np.zeros(act_dim),
                  "Wv": _orthogonal(hidden, 1, rng, .01), "bv": np.zeros(1)}

    def forward(self, obs):
        """obs: (..., obs_dim) -> logits (..., act), value (..., 1)."""
        h1 = np.tanh(obs@self.p["W1"]+self.p["b1"])
        h2 = np.tanh(h1@self.p["W2"]+self.p["b2"])
        return h2@self.p["W3"]+self.p["b3"], h2@self.p["Wv"]+self.p["bv"], (h1, h2)

    def act(self, obs, rng=None, deterministic=False):
        logits, value, _ = self.forward(obs)
        logits = logits-logits.max(axis=-1, keepdims=True)
        p = np.exp(logits)
        p /= p.sum(axis=-1, keepdims=True)
        if deterministic:
            action = p.argmax(axis=-1)
        else:
            action = np.array([rng.choice(self.act_dim, p=pr) for pr in p])
        logp = np.log(np.take_along_axis(p, action[:, None], axis=1)[:, 0]+1e-12)
        return action, logp, value[:, 0]

    def _gradients(self, obs, action, coeff, ret, vf_coef=.5):
        """Backprop for the PPO surrogate and value loss.

        coeff: per-sample d(surrogate)/d(logp) (0 where the clip is active),
        already divided by the batch size; gradients are averaged over samples.
        """
        n = len(obs)
        h1 = np.tanh(obs@self.p["W1"]+self.p["b1"])
        h2 = np.tanh(h1@self.p["W2"]+self.p["b2"])
        logits = h2@self.p["W3"]+self.p["b3"]
        logits -= logits.max(axis=-1, keepdims=True)
        p = np.exp(logits)
        p /= p.sum(axis=-1, keepdims=True)
        onehot = np.zeros_like(p)
        onehot[np.arange(n), action] = 1.
        # policy head: dL/dlogits from logp coefficient, plus entropy bonus
        ent = -(p*np.log(p+1e-12)).sum(axis=1)
        dlogits = coeff[:, None]*(p-onehot)
        dlogits += (0.01/n)*p*(np.log(p+1e-12)+ent[:, None])
        gW3 = h2.T@dlogits
        gb3 = dlogits.sum(axis=0)
        dh2 = dlogits@self.p["W3"].T
        # value head
        v = h2@self.p["Wv"]+self.p["bv"]
        dv = (vf_coef/n)*(v[:, 0]-ret)
        gWv = h2.T@dv[:, None]
        gbv = np.array([dv.sum()])
        dh2 += dv[:, None]@self.p["Wv"].T
        dh2 *= 1.-h2**2
        gW2 = h1.T@dh2
        gb2 = dh2.sum(axis=0)
        dh1 = dh2@self.p["W2"].T*(1.-h1**2)
        gW1 = obs.T@dh1
        gb1 = dh1.sum(axis=0)
        return {"W1": gW1, "b1": gb1, "W2": gW2, "b2": gb2, "W3": gW3, "b3": gb3,
                "Wv": gWv, "bv": gbv}

    def learn(self, batch, rng, clip=.2, lr=3e-4, epochs=4, minibatch=1024, vf_coef=.5):
        obs = batch["obs"].reshape(-1, batch["obs"].shape[-1])
        action = batch["action"].reshape(-1).astype(int)
        logp_old = batch["logp"].reshape(-1)
        value = batch["value"].reshape(-1)
        ret, adv = batch["ret"], batch["adv"]
        n = len(obs)
        indices = np.arange(n)
        m, v_ = ({k: np.zeros_like(x) for k, x in self.p.items()} for _ in range(2))
        beta1, beta2, eps = .9, .999, 1e-8
        stats = {"kl": 0., "clip_frac": 0.}
        steps = 0
        for _ in range(epochs):
            rng.shuffle(indices)
            for start in range(0, n, minibatch):
                idx = indices[start:start+minibatch]
                logits, values, _ = self.forward(obs[idx])
                logits -= logits.max(axis=-1, keepdims=True)
                p = np.exp(logits)
                p /= p.sum(axis=-1, keepdims=True)
                logp = np.log(np.take_along_axis(p, action[idx][:, None], 1)[:, 0]+1e-12)
                ratio = np.exp(logp-logp_old[idx])
                adv_mb = adv[idx]
                unclipped = ratio*adv_mb
                clipped = np.clip(ratio, 1.-clip, 1.+clip)*adv_mb
                use_min = unclipped < clipped
                surrogate = np.where(use_min, unclipped, clipped)
                active = use_min | (unclipped == clipped)
                coeff = ratio*adv_mb*active/len(idx)
                grad = self._gradients(obs[idx], action[idx], coeff, ret[idx], vf_coef)
                steps += 1
                for k in self.p:
                    g = grad[k]
                    m[k] = beta1*m[k]+(1.-beta1)*g
                    v_[k] = beta2*v_[k]+(1.-beta2)*g*g
                    mh = m[k]/(1.-beta1**steps)
                    vh = v_[k]/(1.-beta2**steps)
                    self.p[k] -= lr*mh/(np.sqrt(vh)+eps)
                stats["clip_frac"] += float((~active).mean())
                stats["kl"] += float((logp_old[idx]-logp).mean())
        stats["kl"] /= max(steps, 1)
        stats["clip_frac"] /= max(steps, 1)
        stats["loss"] = float(-surrogate.mean()+vf_coef*np.mean((values[:, 0]-ret[idx])**2))
        return stats

    def save(self, path):
        np.savez(path, obs_dim=self.obs_dim, act_dim=self.act_dim, hidden=self.hidden,
                 **self.p)

    @classmethod
    def load(cls, path):
        data = np.load(path)
        policy = cls(int(data["obs_dim"]), int(data["act_dim"]), int(data["hidden"]))
        for k in policy.p:
            policy.p[k] = data[k]
        return policy


def gae(rewards, values, dones, gamma=.99, lam=.95):
    """rewards/values/dones: (T, n_env) -> advantages and returns, (T, n_env)."""
    t_steps, n_env = rewards.shape
    adv = np.zeros_like(rewards)
    last = np.zeros(n_env)
    for t in reversed(range(t_steps)):
        nonterminal = 1.-dones[t]
        delta = rewards[t]+gamma*values[t+1]*nonterminal-values[t]
        last = delta+gamma*lam*nonterminal*last
        adv[t] = last
    return adv, adv+values[:-1]
