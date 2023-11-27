# iLQR---double-pendulum
Software : 
1. Mathematica for symbolic calculation of inverse dynamics of the linearised  model
2. Matlab for numerical evaluation

Implements a Constrained Iterative LQR controller for an Autonomous Vehicle.
Implemented an iterative Linear Quadratic Regulator (iLQR) algorithm that incorporates constraints in the environment for on-road autonomous motion planning. Since iLQR is based on the theory of dynamic programming, it does not inherently take constraints like obstacles, actuator limits, etc into account. Therefore a Constrained Iterative Linear Quadratic Regulator [1] [2] (CILQR) algorithm is used, which solves constrained optimal control problem in nonlinear systems efficiently. The algorithm is then deployed in an autonomous driving simulator, which will also be used for validation of the project.

Here are the results in a self-developed Python Simulator
Two different behaviors are shown depending on the cost of deviating from the reference trajectory and deviating from the desired speed.
In this first GIF, there is a high cost for deviating from the reference trajectory and hence the ego-vehicle(in red) stays close to the reference path (red line) and does not overtake the NPC vehicle (in gree
