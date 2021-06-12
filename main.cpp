//#include <ct/core/core.h>
#include <iostream>
#include <ct/optcon/optcon.h>  // also includes ct_core
#include <Eigen/Dense>
#include <cmath>
int main(int argc, char** argv)
{
    const size_t state_dim = 4;
    const size_t control_dim = 1;
    
    ct::core::ControlVector<control_dim> u;  // control
    u.setZero();
    
    // parameters of an environment and Cart
    // system CI
    double m = 1.0;
    double M = 5.0; // mass of a Cart
    double L = 2.0; // length
    double g = 9.81;
    double d = 0.01; // coefficient of a friction

    Eigen::Matrix<double, state_dim, state_dim> A;
    A.row(0) << 0.0, 1.0, 0.0, 0.0;
    A.row(1) << 0.0, 0.0, g*m/M, 0.0;
    A.row(2) << 0.0, 0.0, 0.0, 1.0;
    A.row(3) << 0.0, 0.0, g*(m+M)/L/M, 0.0;

    Eigen::Matrix<double, state_dim, control_dim> B;
    B << 0.0, 1.0/M, 0.0, 1.0/L/M;

    Eigen::Matrix<double, state_dim, state_dim> Q;
    Q.row(0) << 1.0, 0.0, 0.0, 0.0;
    Q.row(1) << 0.0, 1.0, 0.0, 0.0;
    Q.row(2) << 0.0, 0.0, 1.0, 0.0;
    Q.row(3) << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix<double, control_dim, control_dim> R;
    R << 0.0001;

    std::cout << "A: " << std::endl << A << std::endl << std::endl;
    std::cout << "B: " << std::endl << B << std::endl << std::endl;
    std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
    std::cout << "R: " << std::endl << R << std::endl << std::endl;

    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;
    bool RisDiagonal = true;
    bool solveRiccatiIteratively = true;
    lqrSolver.compute(Q, R, A, B, K, RisDiagonal, solveRiccatiIteratively);
    std::cout << "LQR gain matrix:" << std::endl << K << std::endl;

    double dt = 0.1; // c
    double T = 3;
    size_t nSteps = 1000;
    ct::core::StateVector<state_dim> x0;  // initial state
    x0.setZero();
    x0(0) = -1.0;
    x0(3) = -0.01; // angle deviation

    ct::core::StateVector<state_dim> x1;  // finish state
    x1.setZero();
    x1(0) = 1.0;
    x1(3) = 0.0;
    // u = -K*(x_finish - x0)   !! возможно, не нужно делать B*K
    // dx/dt = A*x + B* u
    // проверка - посмотреть на графики x(3)=theta (не должен отклоняться от нуль сильно) и x(1) = x (должен пройти от -1 до 1)

    // initial conditions
    const size_t len_x = 30; // =round(T / dt)

    Eigen::Matrix<double, len_x, state_dim> x;
    x.setZero();
   // std::cout << x << std::endl;
    //std::cout << length(x) << std::endl;
    for (size_t i = 0; i < len_x; ++i)
    {
        //std::cout << i << x.row(i) << std::endl;
        if (i < round(len_x / 2))
        {
            x.row(i) << x0(0), x0(1), x0(2), x0(3);
        }
        else
        {
            x.row(i) << x1(0), x1(1), x1(2), x1(3);
        }
    }
    std::cout << x << std::endl;
    double t = 0.0;
    while (t <= T)
    {
        // по всему пространству
        for (size_t i = 0; i < len_x; ++i)
        {
            x(4 * (i + 1)) = x(4 * i) + dt * x(4 * i + 1);
            x(4 * (i + 1) + 1) = x(4 * i + 1) + dt * ( A(6) * x(4 * i + 3))
        }


        t += dt;
    }


    return 0;
}