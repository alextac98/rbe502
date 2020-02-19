%% RBE 502 Homework 3
% By Alex Tacescu
%
% Note: TwoLinkArm file was migrated to this file, to make code look
% cleaner

clc;
clear all;
close all;

% the following parameters for the arm
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;

% we compute the parameters in the dynamic model
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

% Compute M, C, and G variables
symx= sym('symx',[4,1]); 

M = [a+2*b*cos(symx(2)), d+b*cos(symx(2));
    d+b*cos(symx(2)), d];
C = [-b*sin(symx(2))*symx(4), -b*sin(symx(2))*(symx(3)+symx(4)); b*sin(symx(2))*symx(3),0];
G = [m1*g*r1*cos(symx(1))+m2*g*(l1*cos(symx(1))+r2*cos(symx(1)+symx(2)));
    m2*g*r2*cos(symx(1)+symx(2))];


invM = inv(M);
invMC= inv(M)*C;

%% Part 1: Implement a Feedback Controller

q0 = [pi; pi/2];
qf = [0; 0];


%% Part 2: Generate Desired Trajectory
q0_2 = [0; 0];
qf_2 = [pi/3; pi/4];
t0_2 = 0; tf_2 = 10;
v0_2 = [0; 0];
vf_2 = [0; 0];

TwoLinkArmTraj(q0_2, qf_2, v0_2, vf_2, t0_2, tf_2)

%% TODO: GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.
%% You need to specify the initial state, final state (in the state variables of the state space form), the time span.
%% If quintic trajectories are to be generated, consider specify the desired accelerations for all joints.

%% Implement the inverse dynamic control  
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) inverseDC(t,x),[0 tf],x0, options);

%% Plotting the result: please plot both the actual trajectory and the desired trajectory for the your states. example code below.
% figure('Name','Theta_1 under inverse dynamic control');
% plot(T, X(:,1),'r-');
% hold on
% figure('Name','Theta_2 under inverse dynamic control');
% plot(T, X(:,2),'r--');
% hold on