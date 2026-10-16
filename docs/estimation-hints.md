# Mass-spring-damper parameter estimation

The plant is `m x'' + b x' + k x = u` with `theta* = [m, b, k]` unknown. The estimator
reads `x` and `u` only. The method follows slides 45-47 of the estimation deck: the plant,
two repeated-pole filters, and the gradient law are integrated as one ODE with `solve_ivp`.

Run everything from `camslab_offline/`:

    uv run pytest tests/msd
    uv run python -m camslab_offline.msd            # plot of theta(t)
    uv run python -m camslab_offline.msd --no-plot

## Implementation

`msd/online.py` `ode()` is the Python version of `dynamics()` on slide 46.

- The state order is `[x1, x2, xi_x1, xi_x2, xi_u1, xi_u2, m_hat, b_hat, k_hat]`.
- The input `f` is the sum of three sinusoids from slide 46, written in `ode()`.
- The filter output `w = H xi + [0; G] signal` holds the signal passed through
  `1/Lambda`, `s/Lambda`, and `s^2/Lambda`. `phi` is `w_x` reversed, so it is ordered as
  `[m, b, k]`, and `z` is the first entry of `w_f`.
- The plant states `x1` and `x2` simulate the plant. The estimator part uses `x1` and `f`
  only.

## Derivation

1. Laplace transform with zero initial conditions: `(m s^2 + b s + k) X = F`.
2. Divide by `Lambda(s) = (s + lambda)^2`, a stable polynomial of the plant's order:

       m (s^2/Lambda) x + b (s/Lambda) x + k (1/Lambda) x = (1/Lambda) f

3. This is `z = theta*^T phi` with `z = (1/Lambda) f` and
   `phi = [(s^2/Lambda) x, (s/Lambda) x, (1/Lambda) x]`.
4. The gradient law minimizes `J = eps^2 / 2` with `eps = theta_hat^T phi - z`:
   `theta_hat' = -Gamma eps phi`.
5. With `V = theta_tilde^T Gamma^-1 theta_tilde / 2`, `V' = -eps^2 <= 0`. The estimate
   converges to `theta*` when `phi` is persistently exciting.

## Reading

Ioannou and Sun, *Robust Adaptive Control*: section 2.4 (parametric models), section 4.3
(gradient). Ioannou and Fidan, *Adaptive Control Tutorial*, chapters 2 and 3, covers the
same material more briefly.

## Experiments

- Change `--lam` and `--gamma` and see how the convergence time changes.
- Remove two of the three sinusoids from `f` in `ode()` and see which parameters stop
  converging. That is persistent excitation failing.
