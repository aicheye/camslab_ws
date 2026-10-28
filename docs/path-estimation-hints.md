# Circular path identification

This applies the method from the estimation deck to the circle on your second sheet. A
unicycle robot drives near a circle with centre `c` and radius `r`, and measures its
position `p` and the distance to the circle:

    xi = ||p - c|| - r

The estimator recovers `theta* = [r, c_x, c_y]` from `p` and `xi`. The robot is driven by
a known excitation input, which is the open-loop identification case on slide 38. The
circle is fixed, so `c` and `r` are constant, as your sheet's derivation assumes.

`path_circle/circle.py` implements every step below, and its tests pass. The same
estimator runs in the tracking simulator and in Gazebo as `camslab.adaptive`
(see the developer reference).

Run everything from `camslab_standalone/`:

    uv run pytest tests/path_circle
    uv run python -m camslab_standalone.path_circle

## Problem statement

In the form on your first sheet, `x' = f(t, x, u; theta)` and `y = g(t, x, u; theta)`:

| Role | Symbol |
|---|---|
| State `x` | `p_x`, `p_y`, heading `psi` |
| Input `u` | `v`, `w`, from `excitation(t)` |
| Plant `f` | `p' = v [cos psi, sin psi]`, `psi' = w` |
| Output `g` | `p` and `xi = ||p - c|| - r` |
| Parameters `theta` | `[r, c_x, c_y]` |

The parameters appear only in the output `g`. In the mass-spring-damper they appeared in
the plant `f`.

## Derivation

### 1. Parametric model (sheet 2, slides 12-14)

Your sheet ends with

    xi xi' + r xi' = p^T p' - c^T p'

Moving the terms without unknowns to the left gives `z = theta*^T phi`:

    z   = xi xi' - p^T p'
    phi = [-xi', -p_x', -p_y']
    theta* = [r, c_x, c_y]

### 2. Dynamic filtering (slides 15-21)

`xi'` and `p'` are not measured. Following slide 17, both sides are filtered by
`1/Lambda(s)` with `Lambda(s) = s + lambda`, since only first derivatives appear.

- `xi xi' = d/dt (xi^2 / 2)` and `p^T p' = d/dt (||p||^2 / 2)`, so each product is
  `s / Lambda` applied to one measured signal.
- `theta*` is constant, so it passes through the linear filter unchanged:

      z   = s/Lambda (xi^2 / 2) - s/Lambda (||p||^2 / 2)
      phi = -[s/Lambda xi, s/Lambda p_x, s/Lambda p_y]

- Five signals pass through the filter: `xi^2 / 2`, `||p||^2 / 2`, `xi`, `p_x`, and `p_y`.

### 3. Filter state space (slides 22-27)

`filter_matrices(lam, n)` returns `F`, `G` and `H` from slide 25.
`test_filter_matrices_match_slide_47` checks that `n = 2` gives the matrices on slide 47.
For one signal `y`:

    xi_y' = F xi_y + G y
    w_y   = H xi_y + [0; G] y      = [ y / Lambda,  s y / Lambda ]   when n = 1

With `n = 1` each signal has one filter state, so `N_FILTER_STATES = 5`. For `n = 1`,
`F = [-lambda]`, `G = [1]`, and `H = [[1], [-lambda]]`, so `s y / Lambda = -lambda xi_y + y`.

### 4. `ode()` (slides 39-42 and 46)

`ode()` follows the layout on slide 46:

1. Extract the states: plant, filter states, `theta_hat`.
2. Assign the input with `excitation(t)`.
3. Algebraic assignment: `xi` from the true circle (this is the sensor), the filter
   outputs, `z`, `phi` and `z_hat`.
4. Compute the derivatives: plant, filters, and `theta_hat' = -Gamma (z_hat - z) phi`.
5. Combine them in the order of `X`.

Only the sensor line may read `circle`. Everything after it uses `p` and `xi` only.

`test_estimate_converges` checks that the estimate is within 0.05 of `[1, 2, 2]` after
120 s.

## Experiments

- In `excitation()`, set `w = 0`, so the robot drives in a straight line, and see whether
  all three estimates still converge. This tests persistent excitation.
- Change `--lam` and `--gamma` and see how the convergence time changes.

## Open questions

- A general path, where `c` and `r` change as the closest point moves, is not covered.

## Reading

Ioannou and Fidan, *Adaptive Control Tutorial*, chapters 2 and 3.
