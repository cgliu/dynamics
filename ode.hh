#pragma once

///
/// Implement a standard Runge-kutta template
/// dxdt = callFunc(x,u)
///
template <typename callFunc, typename StateVector, typename ControlVector>
StateVector ODE_RK4(callFunc && f, StateVector x, ControlVector u, double dt)
{
    StateVector x_new = x;
    {
        const double half_dt = 0.5 * dt;
        const StateVector k1 = f(x, u);
        const StateVector k2 = f(x + half_dt * k1, u);
        const StateVector k3 = f(x + half_dt * k2, u);
        const StateVector k4 = f(x + dt * k3, u);
        x_new += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    return x_new;
}

///
/// Implement a standard Euler integration template
///
template <typename callFunc, typename StateVector, typename ControlVector>
StateVector ODE_Euler(callFunc && f, StateVector x, ControlVector u, double dt)
{
    StateVector x_new;
    x_new = x + f(x, u) * dt;
    return x_new;
}
