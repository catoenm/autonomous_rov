% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - computeMotorOutputs.m
% Function to calculate motor ouputs - identical logic to arduino code
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
%
% Notes:
%   The sign is flipped when the throttle is adjusted by the calculated 
%   pid values. This is because the MATLAB model uses a conventional 
%   aviation coordiante system wheras the quadcopter firmware uses a 
%   coordinate system with opposite poisitive axis direction.
%
% ---------------------------------------------------------------------- %

function [quadState] = computeMotorOutputs(quadState, quadSetpoints, ...
                                           pid, deltaTPID)
    % error: (roll, pitch, yaw)
    error = quadSetpoints.attitude - quadState.attitude; 

    % output: (roll, pitch, yaw)
    output =   error                   .* pid.p ...
             + quadState.integralError .* pid.i ...
             - quadState.dattitude     .* pid.d; 

    if quadSetpoints.throttle == 0
      quadState.motorOut = [0;0;0;0];
    else
      % Integrate Error
      quadState.integralError = quadState.integralError + error.*deltaTPID;

      % Set outputs 
      quadState.motorOut(1,1) = quadSetpoints.throttle + output(1) + output(2) - output(3);
      quadState.motorOut(2,1) = quadSetpoints.throttle - output(1) + output(2) + output(3);
      quadState.motorOut(3,1) = quadSetpoints.throttle - output(1) - output(2) - output(3);
      quadState.motorOut(4,1) = quadSetpoints.throttle + output(1) - output(2) + output(3);
    end  
end