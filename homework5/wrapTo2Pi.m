function [angle_rad] = wrapTo2Pi(angle_rad)
angle_rad = angle_rad - 2*pi*floor(angle_rad/(2*pi));
end