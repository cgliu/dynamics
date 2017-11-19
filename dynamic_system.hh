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

    StateVector continuous_dynamics(const StateVector & x, const ControlVector & u);

    StateVector discrete_dynamics(const StateVector& x, const ControlVector& u, double dt);

    void print(const StateVector &, const ControlVector&, double);

private:
    Derived& impl() {
        return *static_cast<Derived*>(this);
    }
};

template <typename Derived>
typename DynamicSystem_traits<Derived>::StateVector
DynamicSystem<Derived>::continuous_dynamics(const StateVector & x, const ControlVector & u)
{
    // Have a different function name for implementation to avoid dead loop
    // which happens when you forget to implement a 'virtual' function
    return impl().continuous_dynamics_impl(x, u);
}

template <typename Derived>
typename DynamicSystem_traits<Derived>::StateVector
DynamicSystem<Derived>::discrete_dynamics(const StateVector & x, const ControlVector & u, double dt)
{
    return ODE_RK4(impl().continuous_dynamics_impl,
                   x, u, dt);
}

template <typename Derived>
void DynamicSystem<Derived>::print(const StateVector & x, const ControlVector & u, double t)
{
    std::cout << t << " ";
    std::cout << x.transpose() << " ";
    std::cout << u.transpose() << std::endl;
}
