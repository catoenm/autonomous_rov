% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - noise.m
% Function simulate noise for roll pitch and yaw. Output is in deg/s^2
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
% Notes: 
%
%
% ---------------------------------------------------------------------- %

function [noise] = noise(time, axis)
    switch axis 
        case 1 % Roll
            noise = 2 - rem(rand(1),4); % Random value between -2 and 2 m/s^2 
        case 2 % Pitch
            noise = 2 - rem(rand(1),4); % Random value between -2 and 2 m/s^2 
        case 3 % Yaw 
            noise = 0; 
    end
end