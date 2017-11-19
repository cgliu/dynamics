/////////////////////////////////////////
// Double integrator dynamics template //
/////////////////////////////////////////

#include <array>
#include <Eigen/Core>

#include "dynamic_system.hh"

// Foward declaration, REQUIRED for template code to work
class DoubleIntegrator;

// Template specialization of traits class to define the state, control and output type
template <> struct DynamicSystem_traits<DoubleIntegrator> {
    enum { POS=0,
           VEL,
           x_dim_};
    enum { ACCEL=0,
           u_dim_};
    using StateVector   = Eigen::Matrix<double, x_dim_, 1>;
    using ControlVector = Eigen::Matrix<double, u_dim_, 1>;
};

// Curiously recurring template pattern (CRTP) is used here
class DoubleIntegrator : public DynamicSystem <DoubleIntegrator>
{
public:
    using StateVector   = DynamicSystem_traits<DoubleIntegrator>::StateVector;
    using ControlVector = DynamicSystem_traits<DoubleIntegrator>::ControlVector;

    inline static int x_dim() { return DynamicSystem_traits<DoubleIntegrator>::x_dim_;}
    inline static int u_dim() { return DynamicSystem_traits<DoubleIntegrator>::u_dim_;}

    //Here we use a functor to make it work with function calling that requires non-static functions
    struct
    {
        StateVector operator() (const StateVector & x, const ControlVector & u)
        {
            StateVector dxdt;
            dxdt[0] = x[1];
            dxdt[1] = u[0];
            return dxdt;
        }
    } continuous_dynamics;
};
