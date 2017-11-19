////////////////////////////////////////
// Dynamic system base class template //
////////////////////////////////////////

#include <iostream>
#include "ode.hh"

// Declare templated traits class for Derived
template<typename Derived> struct DynamicSystem_traits {};

template <typename Derived>
class DynamicSystem
{
public:
    using StateVector   = typename DynamicSystem_traits<Derived>::StateVector;
    using ControlVector = typename DynamicSystem_traits<Derived>::ControlVector;

    // Static polymophism
    StateVector continuous_dynamics(StateVector x, ControlVector u);

    StateVector discrete_dynamics(StateVector x, ControlVector u, double dt);

    void print(StateVector, ControlVector, double);
};

template <typename Derived>
typename DynamicSystem_traits<Derived>::StateVector
DynamicSystem<Derived>::continuous_dynamics(StateVector x, ControlVector u)
{
    return static_cast<Derived *> (this)->continuous_dynamics(x, u);
}

template <typename Derived>
typename DynamicSystem_traits<Derived>::StateVector
DynamicSystem<Derived>::discrete_dynamics(StateVector x, ControlVector u, double dt)
{
    return ODE_RK4(static_cast<Derived *> (this)->continuous_dynamics,
                   x, u, dt);
}

template <typename Derived>
void DynamicSystem<Derived>::print(StateVector x, ControlVector u, double t)
{
    std::cout << t << " ";
    std::cout << x.transpose() << " ";
    std::cout << u.transpose() << std::endl;
}
