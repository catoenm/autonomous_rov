% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - mach1Properties.m
% Function to generate mach1Geometry file
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
% Notes: 
%
%   The origin is located at the center of the quadcopter on the plane that
%   the legs touch the ground (ie. the point on the ground centered below 
%   the frame). 
%
%   Propeller identification scheme: 
%       FR - Front Right
%       FL - Front Left
%       BR - Back Right
%       BL - Back Left
% 
%   Units: 
%       Distance: m
%       Mass: Kg
%
% ---------------------------------------------------------------------- %

function mach1Properties()
    
    % Set loop time to 82Hz (Mach 1 flight controller as of August 2016)
    LOOP_TIME = 0.012195;

    % Center of Gravity
    %COG = [0, 0, -0.1078];
    COG = [-0.0005, -0.0010, -0.1078];

    % Mass
    MASS = 1.03587;

    % Properller positions
    FR_POS = [0.1762, 0.1762, -0.1417];
    FL_POS = [0.1762, -0.1762, -0.1417];
    BL_POS = [-0.1762, -0.1762, -0.1417];
    BR_POS = [-0.1762, 0.1762, -0.1417];
    
    PROP_RELATIVE_POS = [FR_POS-COG; FL_POS-COG; BL_POS-COG; BR_POS-COG];

    % Moments of Inertia (about center of gravity)
    IXX = 0.010919933;
    IYY = 0.011844647;
    IZZ = 0.022089975;
    
    save('mach1Properties'); 
end
