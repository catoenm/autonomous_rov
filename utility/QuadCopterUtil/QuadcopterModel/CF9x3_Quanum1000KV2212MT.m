% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - CF9x3_Quanum1000KV2212MT.m
% Function to generate property file for 9x3 carbon fiber prop with Quanum 
% MT Series 2212 1000KV motor. 
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
%
% Notes:
%
%   All curve coefficients are stored in descending power (like polyfit)
%
% ---------------------------------------------------------------------- %

function CF9x3_Quanum1000KV2212MT()
    
    % Center of Gravity
    PROP_MASS = 0.001;
    
    % Torque - rpm curve coefficients;
    RPM_TO_TORQUE = [3.76866995065736e-09,-5.94955569962960e-06,0.00729215772784099];
    
    % Thrust - rpm curve coefficients; 
    RPM_TO_THRUST = [2.45463013670666e-08,0.000316302688232674,-0.451441446845131];
    
    % Thrust - rpm curve coefficients; 
    THROTTLE_TO_RPM = [0.240749414519906,94.0732066961576,995.011709601872];
    
    save('CF9x3_Quanum1000KV2212MT'); 
end
