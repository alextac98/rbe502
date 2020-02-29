%% RBE 502 Homework 4
% By Alex Tacescu

%%
clc;
clear;

%% Part 1: System Dynamic Model
%
% $\dot{x} = v * cos(\theta)$
%
% $\dot{y} = v * sin(\theta)$
%
% $\dot{\theta} = \omega$
%
% $v \rightarrow$ linear velocity of the car
%
% $\omega \rightarrow$ angular velocity of the car
%

syms x y theta t v omega

x_dot = v * cos(theta);
y_dot = v * sin(theta);
theta_dot = omega;

% Input constants
v_r = 10;
y_r = 2;

% Set up desired
x_d = v_r * t;
y_d = y_r;
theta_d = 0;
omega_d = 0;
v_d = v_r;

e_x =  [x - x_d;
        y- y_d;
        theta - theta_d]
e_u =  [v_r - v_d;
        omega - omega_d]

%% Part 2: Jacobian Linearization of Dynamic System
%
% $\dot{\vec{e_x}} = \frac{\partial f}{\partial x} \vec{e_x} + \frac{\partial f}{\partial u} \vec{e_u}$
%
% $A = \frac{\partial f}{\partial x}$
%
% $B = \frac{\partial f}{\partial u}$


A = [0, 0, -v*sin(theta);
     0, 0,  v*cos(theta);
     0, 0,            0];

B = [cos(theta), 0;
     sin(theta), 0;
              0, 1];
          
e_x_dot = A * e_x + B * e_u

%% Part 3: Feedback Controller for Steering Control
% 
% $e_u = u - u_d = -k * e_x$
% 
% $u = -k(e_x) + u_d$

k = [1, 1, 1;
     1, 1, 1]; 

u_d = [v_d; omega_d];

u = -k * e_x + [v_d; omega_d]



%% Part 4: Controller + Plotting Trajectory
% Sorry... Still can't figure out how to use ode45....


param = [8; 0];

sampling_time = 0.1;
tspan = 0:sampling_time:10;

[T,X] = ode45(@(t,x_in) ode_dubins(t,x_in, param), tspan, 0, param);

figure
plot(T,X(:,1),'LineWidth',4);
xlabel('t');
ylabel('x');

figure
plot(T,X(:,2),'LineWidth',4);
xlabel('t');
ylabel('y');

figure
plot(T,X(:,3),'LineWidth',4);
xlabel('t');
ylabel('theta');

figure
plot(X(:,1), X(:,2))














 
