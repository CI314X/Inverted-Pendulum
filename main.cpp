//#include <ct/core/core.h>
#include <iostream>
#include <ct/optcon/optcon.h>  // also includes ct_core
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t state_dim = 4;
    const size_t control_dim = 1;
    // create a state
    ct::core::StateVector<state_dim> x;
    x.setZero(); // we initialize it at 0
    x(3) = 0.01;

    ct::core::ControlVector<control_dim> u;
    u.setZero();
    
    // create our mass point instance
    double m = 1.0;
    double M = 5.0;
    double L = 2.0;
    double g = -9.81;
    double d = 0.01;
    double dt = 0.001;
    double t = 0.0;
    size_t nSteps = 100;

    Eigen::Matrix<double, state_dim, state_dim> A;
    A.row(0) << 0.0, 1.0, 0.0, 0.0;
    A.row(1) << 0.0, 0.0, g*m/M, 0.0;
    A.row(2) << 0.0, 0.0, 0.0, 1.0;
    A.row(3) << 0.0, 0.0, g*(m+M)/L/M, 0.0;

    Eigen::Matrix<double, state_dim, control_dim> B;
    B << 0.0, 1.0/M, 0.0, 1.0/L/M;

    //ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;

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
    // design the LQR controller
    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;
    lqrSolver.compute(Q, R, A, B, K);
    std::cout << "LQR gain matrix:" << std::endl << K << std::endl;

    //integrator.integrate_n_steps(x, t0, nSteps, dt);
    // print the new state
    //std::cout << "state after integration: " << x.transpose() << std::endl;
    return 0;
}