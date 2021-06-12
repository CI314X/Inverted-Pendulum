#include <iostream>
#include <ct/optcon/optcon.h>  // also includes ct_core
#include <Eigen/Dense>
//#include <cmath>
#include <fstream>

int main(int argc, char** argv)
{
    // system SI

    const size_t state_dim = 4;
    const size_t control_dim = 1;

    ct::core::ControlVector<control_dim> u;  // control
    u.setZero();
    
    // parameters of an environment and Cart

    double m = 1.0;
    double M = 5.0; // mass of the Cart
    double L = 2.0; // length
    double g = 9.81;
    double d = 1.0; // coefficient of friction

    Eigen::Matrix<double, state_dim, state_dim> A;
    A.row(0) << 0.0, 1.0, 0.0, 0.0;
    A.row(1) << 0.0, -d / M, g * m / M, 0.0;
    A.row(2) << 0.0, 0.0, 0.0, 1.0;
    A.row(3) << 0.0, -d / M / L, g * (m + M) / M / L, 0.0;

    Eigen::Matrix<double, state_dim, control_dim> B;
    B << 0.0, 1.0 / M, 0.0, 1.0 / M / L;              

    Eigen::Matrix<double, state_dim, state_dim> Q;
    Q.row(0) << 1.0, 0.0, 0.0, 0.0;
    Q.row(1) << 0.0, 1.0, 0.0, 0.0;
    Q.row(2) << 0.0, 0.0, 1.0, 0.0;
    Q.row(3) << 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix<double, control_dim, control_dim> R;
    R << 0.001;

    //std::cout << "A: " << std::endl << A << std::endl << std::endl;
    //std::cout << "B: " << std::endl << B << std::endl << std::endl;
    //std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
    //std::cout << "R: " << std::endl << R << std::endl << std::endl;

    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;

    bool RisDiagonal = true;
    bool solveRiccatiIteratively = true;
    lqrSolver.compute(Q, R, A, B, K, RisDiagonal, solveRiccatiIteratively);

    //std::cout << "LQR gain matrix:" << std::endl << K << std::endl;
    //std::cout << (A - B * K).eigenvalues() << std::endl << std::endl;

    ct::core::StateVector<state_dim> x0;  // initial state
    x0 << atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]);

    //x0(0) = 0.0;
    //x0(1) = 0.0;
    //x0(2) = 0.0; // angle deviation
    //x0(3) = 0.5;
    std::cout << "Initial state     : " << x0(0) << "\t" << x0(1) << "\t" << x0(2) << "\t" << x0(3) << std::endl;
    ct::core::StateVector<state_dim> x1;  // final state
    //x1(0) = 1.0;
    x1(0) = atof(argv[5]);
    x1(1) = 0.0;
    x1(2) = 0.0;
    x1(3) = 0.0;
    std::cout << "Wanted Final state: " << x1(0) << "\t" << x1(1) << "\t" << x1(2) << "\t" << x1(3) << std::endl;

    double t = 0.0;
    double dt = 0.1;
    
    Eigen::VectorXd y(state_dim); // current_state (t)
    Eigen::VectorXd yn(state_dim); // future_state (t + dt)

    std::ofstream fout; // file
    fout.open("out1.txt"); 
    fout << "t\tx\tx_dot\ttheta\ttheta_dot\n";

    double eps = 1e-4;  // accuracy of the final position for L2 norm
    y = x0;
    while ((y - x1).squaredNorm() >= eps)
    {
        fout << t << "\t" << std::setprecision(4) << y(0) << "\t" << y(1) << "\t" << y(2) << "\t" << y(3) << "\n";
        
        yn = y + dt * (A * y - B * K * (y - x1));
        y = yn;
        t += dt;
    }

    std::cout << "Stabilization time: " << t << std::endl;
    std::cout << "Final state: " << std::setprecision(6) << y(0) << "\t" << y(1) << "\t" << y(2) << "\t" << y(3) << std::endl;
    fout.close(); 
    return 0;
}