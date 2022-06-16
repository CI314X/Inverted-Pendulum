# Inverted-Pendulum
Test problem for DragonTreeLabs. This code emulates the Linear Quadratic Regulator (LQR) мontrol for the Inverted Pendulum on a Cart. 

### Compile:
```console
user@user:~$ ./compiler
```

### Run:
```console
user@user:~$ ./test x1 x2 x3 x4 x5
```
where x1 - start x-coordinate, x2 - x_dot (initial velocity), x3 - theta (initial angle), x4 - theta_dot, x5 - final x-coordinate.

Libraries - control-toolbox and OpenCV.
### Example of run
```console
user@user:~$ ./test 0.0 0.0 -0.1 0.3 200.0
```

![name](https://github.com/CI314X/Inverted-Pendulum/blob/master/demo.MOV)

<!-- <video width="320" height="240" controls>
  <source src="demo.MOV" type="video/mp4">
</video>
 -->
