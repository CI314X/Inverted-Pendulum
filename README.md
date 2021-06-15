# Inverted-Pendulum
Test problem for DragonTreeLabs

Compile: $ g++ main.cpp -o test -I /usr/include/eigen3/ -I /usr/local/include/opencv4/ -lopencv_core -lopencv_imgproc -lopencv_highgui

Run: $ ./test x1 x2 x3 x4 x5

x1 - start x, x2 - x_dot, x3 - theta, x4 - theta_dot, x5 - final x.

Libraries - control-toolbox and OpenCV.
