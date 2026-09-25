# Integrating the plant: hints and reading

The plant is

    dp/dt = v q
    dq/dt = w S q,    S = [[0, -1], [1, 0]]

`camslab_sim/simulate.py` samples the controller every `dt` and holds `v` and `w` constant
until the next sample (zero-order hold). This is also what the real robot does: the
ROS controller publishes `cmd_vel` at 50 Hz and the car holds each command. So
`step(state, v, w, dt)` only has to solve the ODE over one interval with constant inputs.

## Four ways to write `step`, in order of effort

1. **Forward Euler.** `x_next = x + dt * f(x)`. Ten minutes of work. Before trusting it,
   compute `||q_next||^2` by hand using `q^T S q = 0`. The result explains why
   `test_heading_stays_unit_length` fails for plain Euler.
2. **RK4.** Write a generic `rk4(f, x, dt)` on the stacked vector `[px, py, qx, qy]`.
   Fourth-order accurate. `||q||` still drifts, but by about 1e-11 over the test.
3. **`scipy.integrate.solve_ivp`.** Call it over `[0, dt]` inside `step`, or drop the
   fixed-step loop and integrate the whole closed loop in one call with the controller
   inside the right-hand side. For the one-call version, look at `max_step` and
   `rtol`/`atol`.
4. **Exact solution.** With `w` constant, `dq/dt = w S q` is linear with constant
   coefficients, so `q(t) = expm(w t S) q(0)`. Use `S^2 = -I` in the power series of the
   matrix exponential and see which familiar 2x2 matrix comes out. Then integrate
   `dp/dt = v q(t)` in closed form, and treat `w -> 0` as a separate case. This version
   keeps `||q|| = 1` to machine precision and carries over to C++ with no library.

## Keeping `||q|| = 1`

Options: renormalise `q` after every step, or use the exact solution above. The reason
option 4 preserves the norm is that it moves `q` along the rotation group SO(2) instead
of along a straight line in R^2. "Geometric integrator" is the search term.

## Choosing `dt`

For a linear error equation `de/dt = -k e`, forward Euler is stable only for
`dt < 2 / k` and accurate for `dt * k << 1`. Linearise the closed loop about the
reference to find the rates that play the role of `k` for the tracking law. The same
limit applies to the 50 Hz rate of `controller`.

## C++

A hand-written RK4 or the exact solution needs only `<cmath>`. Boost.Odeint is the
library option.

## Reading

- SciPy `solve_ivp`: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html
- Runge-Kutta methods: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
- Matrix exponential: https://en.wikipedia.org/wiki/Matrix_exponential
- Unicycle, differential-drive and car kinematics: LaValle, *Planning Algorithms*,
  section 13.1.2, http://lavalle.pl/planning/
- Norm-preserving integration: Hairer, Lubich, Wanner, *Geometric Numerical
  Integration*, chapter IV (the library has it)
- Boost.Odeint: https://www.boost.org/doc/libs/release/libs/numeric/odeint/
- Ackermann constraint (the Gazebo car): a car with wheelbase `L` and steering angle `delta`
  turns at `w = v tan(delta) / L`, so `|w| <= |v| tan(delta_max) / L` and `w` is zero
  when `v` is zero.
