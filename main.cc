///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Curiously Recurring Template Pattern (CRTP) is a good way to avoid the inefficiency                   //
// of dynamic polymorphisim of using virtual functions. It supports static polymorphism at compile time. //
// It is both efficient and elegant.                                                                     //
//                                                                                                       //
// This is a example code of CRTP for dynamic system simulation                              //
///////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "double_integrator.hh"

int main()
{
    using StateVector  = DoubleIntegrator::StateVector;
    using ControlVector= DoubleIntegrator::ControlVector;

    DoubleIntegrator model;
    StateVector x{0,0};
    ControlVector u{0.1};

    double t = 0;
    while(t<10)
    {
        model.print(x,u,t);
        x = model.discrete_dynamics(x, u, 0.1);
        t += 0.1;
    }
}
