% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - rkAngVelocity.m
% Function to help generate K values in 4th order RK
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
%
% Notes:
%
%   Function is currently trivial to allow for a more complex model in the
%   future. 
%
% ---------------------------------------------------------------------- %

function [omega] = rkAngVelocity(t, y) 
    omega = y;
end