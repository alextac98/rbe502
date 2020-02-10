%% RBE 502 Homework 2
% By Alex Tacescu

%%
clc; clear;
%% Question 1: State Space Form
%
% $m * \ddot{z} + \lambda * \dot{z} + k * z = 0$
%
% $\vec{x} = \left[\matrix{z \cr \dot{z}} \right]$
%
% $\dot{\vec{x}} = \left[\matrix{x_2 \cr \frac{-\lambda*x_2 -
% k*x_1}{m}}\right]$

%% Question 2: Dynamic Equation in State Space Form
% Given F = u:
%
% $m * \ddot{z} + \lambda * \dot{z} + k * z = u$
%
% $\vec{x} = \left[\matrix{z \cr \dot{z}} \right]$
%
% $\dot{\vec{x}} = \left[\matrix{x_2 \cr \frac{u-\lambda*x_2 -
% k*x_1}{m}}\right]$

%% Question 3: Is the System Controllable?
% Symbolically:
%
% $\dot{x_1} = x_2$
%
% $\dot{x_2} = \frac{u}{m} - \frac{\lambda * x_2}{m} - \frac{k*x_1}{m}$
%
% $\dot{x} = \left[\matrix{0 & 1 \cr \frac{-k}{m} & \frac{-/lambda}{m}} \right] x +
% \left[\matrix{0 \cr \frac{1}{m}} \right]u$
%
% After substituting values: $k = 2, m = 5, \lambda = 1$
%
% $\dot{x} = \left[\matrix{0 & 1 \cr \frac{-2}{5} & \frac{-1}{5}} \right] x +
% \left[\matrix{0 \cr \frac{1}{5}} \right]u$
%
% The system is controllable, because the system is reachable, since the matrix
% $\left[\matrix{B & A*B} \right]$
% is full rank, as proven below

A = [   0,    1;
     -0.4, -0.2];
B = [0; 0.2];

matrix_rank = rank([B, A*B])

%% Question 4: Design a Setpoint Controller
% Since system is stable at the origin, we can ommit $-kx$ from the input
% function
%
% $\dot{x} = A * x_e + B (k_r * r)$
%
% $A*x_e + B (k_r * r) = 0$ && $r = c*x_e$
%
% $x_e = -B*k_r*r*A^{-1}$
%
% $r = -c * A^{-1} * B * k_r * r$
%
% $k_r = -(c * A^{-1} * B)^{-1}$

c = [1, 0];
y_r = 5;

k_r = -inv(c * inv(A) * B)

u = k_r * y_r

%%
% Plugging $u$ back into the equation above, we get:
%
% $\dot{x} = \left[\matrix{0 & 1 \cr \frac{-2}{5} & \frac{-1}{5}} \right] \vec{x} +
% \left[\matrix{0 \cr 2} \right]$
%

%% Question 5: Implement a Setpoint Controller in Matlab
% Test 1:
x0 = 0;
v0 = 0;
[v_out, a_out] = Controller(x0, v0);
disp("Test 1:");
disp("Output Velocity: " + v_out);
disp("Output Acceleration: " + a_out);
%% 
% Test 2:
x0 = -10;
v0 = 16;
[v_out, a_out] = Controller(x0, v0);
disp("Test 2:");
disp("Output Velocity: " + v_out);
disp("Output Acceleration: " + a_out);

%%
% Test 3: Test to make sure controller stabilizes around $x_0 = 5$
x0 = 5;
v0 = 0;
[v_out, a_out] = Controller(x0, v0);
disp("Test 3:");
disp("Output Velocity: " + v_out);
disp("Output Acceleration: " + a_out);

%%
% Test 4: 
x0 = 5;
v0 = 5;
[v_out, a_out] = Controller(x0, v0);
disp("Test 4:");
disp("Output Velocity: " + v_out);
disp("Output Acceleration: " + a_out);


function [v, a] = Controller(x0, v0)
    out = [0, 1; -0.4, -0.2] * [x0; v0] + [0; 2];
    v = out(1);
    a = out(2);
end








