# Mass-spring-damper parameter estimation: what to write

The plant is `m x'' + b x' + k x = u` with `theta* = [m, b, k]` unknown. The estimator may
read `x` and `u` only. The method follows slides 45-47 of the estimation deck: the plant,
two repeated-pole filters and the gradient law are integrated as one ODE with `solve_ivp`.

Run everything from `camslab_estimation/`:

    uv run pytest                                  # 2 tests, both fail until ode() is written
    uv run python -m camslab_estimation            # plot of theta(t)
    uv run python -m camslab_estimation --no-plot

## Your stub

`online.py` `ode()`: the Python version of `dynamics()` on slide 46. `filter_matrices()`
and `run()` are complete. Checked by `tests/test_online.py`.

- The state order is `[x1, x2, xi_x1, xi_x2, xi_u1, xi_u2, m_hat, b_hat, k_hat]`.
- `plant.excitation(t)` is the input `u`. The deck calls it `f`.
- The filter output `w = H xi + [0; G] signal` holds the signal passed through
  `1/Lambda`, `s/Lambda` and `s^2/Lambda`. Work out which entry is which before you build
  `phi` and `z`, and order `phi` to match `[m, b, k]`.
- The plant states `x1` and `x2` are only for simulating the plant. The estimator part
  may use `x1` and `u` only.

## Reading

Ioannou and Sun, *Robust Adaptive Control*: section 2.4 (parametric models), section 4.3
(gradient). Ioannou and Fidan, *Adaptive Control Tutorial*, chapter 2 and 3, covers the
same material more briefly.

## After the tests pass

- Change `--lam` and `--gamma` and see how the convergence time changes.
- Remove two of the three sinusoids in `plant.excitation()` and see which parameters stop
  converging. That is persistent excitation failing.
