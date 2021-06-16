#include <iostream>
#include <ct/optcon/optcon.h> // also includes ct_core
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>

#define w 400

void MyPole(cv::Mat img, cv::Point2d center);
void MyCart(cv::Mat img, cv::Point2d left, cv::Point2d right);
void MyLine(cv::Mat img, cv::Point2d start, cv::Point2d end);
void draw_image(cv::Mat image, double x, double theta, char name_image[]);

int main(int argc, char **argv)
{
  // system SI
  const size_t state_dim = 4;
  const size_t control_dim = 1;

  ct::core::ControlVector<control_dim> u; // control
  u.setZero();

  // parameters of an environment and Cart

  double m = 1.0;  // mass of the pole                           video - 1.0
  double M = 5.0;  // mass of the Cart                           video - 5.0
  double L = 2.0;  // length of the pole                         video - 2.0
  double g = 9.81; // acceleration due to gravity               video - 10.0
  double d = 1.0;  // coefficient of friction                    video - 1.0

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
  Q.row(2) << 0.0, 0.0, 30.0, 0.0;
  Q.row(3) << 0.0, 0.0, 0.0, 30.0;

  Eigen::Matrix<double, control_dim, control_dim> R;
  R << 1.1;

  ct::optcon::LQR<state_dim, control_dim> lqrSolver;
  ct::core::FeedbackMatrix<state_dim, control_dim> K;

  bool RisDiagonal = true;
  bool solveRiccatiIteratively = true;
  lqrSolver.compute(Q, R, A, B, K, RisDiagonal, solveRiccatiIteratively);

  ct::core::StateVector<state_dim> x0; // initial state - x, x_dot, theta, theta_dot
  x0 << atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]);
  std::cout << "Initial state     : " << x0(0) << "\t" << x0(1) << "\t" << x0(2) << "\t" << x0(3) << std::endl;

  ct::core::StateVector<state_dim> x1; // final state - x, x_dot, theta, theta_dot
  x1 << atof(argv[5]), 0.0, 0.0, 0.0;
  std::cout << "Wanted Final state: " << x1(0) << "\t" << x1(1) << "\t" << x1(2) << "\t" << x1(3) << std::endl;

  double t = 0.0;
  double dt = 0.1;

  Eigen::VectorXd y(state_dim);  // current_state (t)
  Eigen::VectorXd yn(state_dim); // future_state (t + dt)

  std::ofstream fout; // file
  fout.open("out1.txt");
  fout << "t\tx\tx_dot\ttheta\ttheta_dot\tcontrol\n";

  double eps = 1e-4; // accuracy of the final position for L2 norm
  y = x0;

  char name_image[] = "Cart";
  cv::Mat image;

  while ((y - x1).squaredNorm() >= eps)
  {
    yn = y + dt * (A * y - B * K * (y - x1));
    fout << t << "\t" << std::setprecision(4) << y(0) << "\t" << y(1) << "\t" << y(2) << "\t" << y(3) << "\t" << -K * (y - x1) << "\n";
    y = yn;
    t += dt;

    draw_image(image, y(0), y(2), name_image);

    cv::waitKey(1500); // time betweeen different states
  }
  fout.close();
  std::cout << "Stabilization time: " << t << std::endl;
  std::cout << "Final state: " << std::setprecision(6) << y(0) << "\t" << y(1) << "\t" << y(2) << "\t" << y(3) << std::endl;
  cv::waitKey(0);
  return 0;
}

void MyCart(cv::Mat img, cv::Point2d left, cv::Point2d right)
{
  cv::rectangle(img,
                left,
                right,
                cv::Scalar(255, 0, 255),
                cv::FILLED,
                cv::LINE_8);
}
void MyPole(cv::Mat img, cv::Point2d center)
{
  cv::circle(img,
             center,
             w / 40,                  //radius
             cv::Scalar(255, 0, 255), // color
             //Scalar( a, b, c )
             //Blue = a, Green = b and Red = c
             cv::FILLED,
             cv::LINE_8);
}
void MyLine(cv::Mat img, cv::Point2d start, cv::Point2d end)
{
  int thickness = 1;
  int lineType = cv::LINE_8;
  cv::line(img,
           start,
           end,
           cv::Scalar(255, 0, 255),
           thickness,
           lineType);
}

void draw_image(cv::Mat image, double x, double theta, char name_image[])
{
  double LENGTH_OF_CART = w / 5;
  double HEIGHT_OF_CART = w / 20;
  double LENGTH_OF_POLE = w / 5;
  double CENTER_OF_PICTURE_X = w / 2;
  double CENTER_OF_PICTURE_Y = w / 2;

  cv::Point2d pole_center = cv::Point2d(0.0, 0.0);
  cv::Point2d cart_left = cv::Point2d(0.0, 0.0);
  cv::Point2d cart_right = cv::Point2d(0.0, 0.0);
  cv::Point2d pole_bottom = cv::Point2d(0.0, 0.0); // нижний конец палки

  pole_center.x = (x + CENTER_OF_PICTURE_X) + LENGTH_OF_POLE * sin(theta); // смещение на w/2
  pole_center.y = CENTER_OF_PICTURE_Y - (0.5 * HEIGHT_OF_CART + LENGTH_OF_POLE * cos(theta));
  pole_bottom.x = x + CENTER_OF_PICTURE_X;
  pole_bottom.y = CENTER_OF_PICTURE_Y - 0.5 * HEIGHT_OF_CART;

  cart_left.x = x + CENTER_OF_PICTURE_X - 0.5 * LENGTH_OF_CART;
  cart_left.y = CENTER_OF_PICTURE_Y - 0.5 * HEIGHT_OF_CART;
  cart_right.x = x + CENTER_OF_PICTURE_X + 0.5 * LENGTH_OF_CART;
  cart_right.y = CENTER_OF_PICTURE_Y + 0.5 * HEIGHT_OF_CART;
  image = cv::Mat::zeros(w, w, CV_8UC3);

  MyPole(image, pole_center);
  MyLine(image, pole_bottom, pole_center);
  MyCart(image, cart_left, cart_right);
  cv::namedWindow(name_image, cv::WINDOW_NORMAL);
  cv::resizeWindow(name_image, 600, 600);
  cv::imshow(name_image, image);
  cv::moveWindow(name_image, 800, 200);
}
