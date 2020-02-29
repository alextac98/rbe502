function dz = ode_dubins(t,z, theta,param)
% use z for [x,y,theta]
dz =zeros(3,1);

%% TODO: Here is the code for control input.
%%% the controller needs to provide control input: linear velocity and
%%% steering angle: v, delta

% step 1: calculate the value of deviation variables given the
% desired state and input and the actual state, exacted from z.

% step 2: based on the feedback controller, calculate the e_u=
% [e_v, e_w].

% step 3: based on the relation between u and e_u, derive the
% desired input v, and w.

v = param(1);
omega = param(2);

dz(1) = v * cos(z(1));
dz(2) = v * sin(z(1));
dz(3) = omega;
end

