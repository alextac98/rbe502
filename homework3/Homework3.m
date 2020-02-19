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
