function output = TwoLinkArmTraj(q0, qf, v0, vf, t0, tf)
%TWOLINKARMTRAJ Cubic Polynomial function to determine constants for a
%trajectory

q_vector = [q0; v0; qf; vf];

a_matrix = [1, t0, t0^2,   t0^3;
            0,  1, 2*t0, 3*t0^2;
            1, tf, tf^2,   tf^3;
            0,  1, 2*tf, 3*tf^2];

output = linsolve(a_matrix, q_vector);

end

