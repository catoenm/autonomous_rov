% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - main.m
% Script to simulate quadcopter flight charactaristics
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
%
% Notes:
%
%   Propeller identification scheme: 
%       Motor 1: FR - Front Right
%       Motor 2: FL - Front Left
%       Motor 3: BL - Back Left
%       Motor 4: BR - Back Right
% 
%   Units: 
%       Distance: meters
%       Mass: kilograms
%       Time: seconds
%
%   Properties: 
%       All quadcopter/motor/properller specific properties are loaded from
%       external files (modular software for easy tranfering to Mach 2,
%       Mach 3, etc...)
%
%   Coordinate System
%       A conventional aviation coordnate system is used (North East Down)
%       http://www.chrobotics.com/library/understanding-euler-angles
%
% ---------------------------------------------------------------------- %

clear all;
close all;

% ------------------------------- Config ------------------------------- %

SIMULATION_TIME = 1; 
LOG_COARSE = true; 
LOG_FINE = true;
ADD_NOISE = false;
PLOT_ATTITUDE = true;

% Override
if PLOT_ATTITUDE
    LOG_FINE = true; % Assert true since plot_atttitude requires fine log
end

% ------------------------------ Constants ----------------------------- %

RAD_TO_DEG = 57.2958;

% Step size for 4th order RK
DELTA_T_RK = 0.001;

% ------------ Load mechanical data and motor/prop behaviour ----------- %

load('mach1Properties.mat'); % quad mechanical properties
load('CF9x3_Quanum1000KV2212MT'); % motor and propeller behaviour

% -------------------------- Initial conditions ------------------------ %

time = 0; 
prevTimePID = 0; 

% Rotations: attitude   = (roll,  pitch,  yaw )
%            dattitude  = (droll, dpitch, dyaw)
%            ddattitude = (ddroll, ddpitch, ddyaw)
quadState.attitude = [5,5,0];
quadState.dattitude = [90,90,10];
quadState.ddattitude = [NaN,NaN,NaN];

% Translation: position   = (x,  y,  z )
%              dposition  = (dx, dy, dz)
%              ddposition = (ddx, ddy, ddz)
quadState.position = [0,0,0];
quadState.dposition = [0,0,0];
quadState.ddposition = [0,0,0];

% Integral error
quadState.integralError = [0,0,0];

% Add ICs to logs
if LOG_COARSE
    i = 1;
    log.coarse(i).time = time;
    log.coarse(i).roll = quadState.attitude(1);
    log.coarse(i).pitch = quadState.attitude(2);
    log.coarse(i).yaw = quadState.attitude(3);
    log.coarse(i).x = quadState.position(1);
    log.coarse(i).y = quadState.position(2);
    log.coarse(i).z = quadState.position(3);
    log.coarse(i).droll = quadState.dattitude(1);
    log.coarse(i).dpitch = quadState.dattitude(2);
    log.coarse(i).dyaw = quadState.dattitude(3);
    log.coarse(i).dx = quadState.dposition(1);
    log.coarse(i).dy = quadState.dposition(2);
    log.coarse(i).dz = quadState.dposition(3);
    log.coarse(i).ddroll = quadState.ddattitude(1);
    log.coarse(i).ddpitch = quadState.ddattitude(2);
    log.coarse(i).ddyaw = quadState.ddattitude(3);
    log.coarse(i).ddx = quadState.ddposition(1);
    log.coarse(i).ddy = quadState.ddposition(2);
    log.coarse(i).ddz = quadState.ddposition(3);
    i = i+1;
end

if LOG_FINE
    j = 1; 
    log.fine(j).time = time;
    log.fine(j).roll = quadState.attitude(1);
    log.fine(j).pitch = quadState.attitude(2);
    log.fine(j).yaw = quadState.attitude(3);
    log.fine(j).x = quadState.position(1);
    log.fine(j).y = quadState.position(2);
    log.fine(j).z = quadState.position(3);
    log.fine(j).droll = quadState.dattitude(1);
    log.fine(j).dpitch = quadState.dattitude(2);
    log.fine(j).dyaw = quadState.dattitude(3);
    log.fine(j).dx = quadState.dposition(1);
    log.fine(j).dy = quadState.dposition(2);
    log.fine(j).dz = quadState.dposition(3);
    log.fine(j).ddroll = quadState.ddattitude(1);
    log.fine(j).ddpitch = quadState.ddattitude(2);
    log.fine(j).ddyaw = quadState.ddattitude(3);
    log.fine(j).ddx = quadState.ddposition(1);
    log.fine(j).ddy = quadState.ddposition(2);
    log.fine(j).ddz = quadState.ddposition(3);
    j = j+1;
end

% ----------------------------- Setpoints ------------------------------ %

% Values broadcasted from controller 
% Attitude: (roll, pitch, yaw)
quadSetpoints.attitude = [0,0,0];
quadSetpoints.throttle = 60;

% Motor Outputs: (FR; FL; BL; BR)
quadState.motorOut = [0;0;0;0];

% ----------------------------- PID Values ----------------------------- %

% PID constants
pid.p = [0.6,0.6,10];
pid.i = [0.0005,0.0005,0.0005];
pid.d = [0.1,0.1,0.5];

% ---------------------------- Simulation ------------------------------ %

while time < SIMULATION_TIME
    % Solve for forces at propellers

    deltaTPID = time - prevTimePID;
    prevTimePID = time;

    [quadState] = computeMotorOutputs(quadState,quadSetpoints,pid,deltaTPID);
    propSpeed = polyval(THROTTLE_TO_RPM, quadState.motorOut);
    propThrust = polyval(RPM_TO_THRUST, propSpeed);
    propTorque = polyval(RPM_TO_TORQUE, propSpeed) .* [-1;1;-1;1];

    % Solve for accelerations 
    % ddattitude: (roll, pitch, yaw)
    % For roll and pitch: 
    %   angularAccel = sumForFourPropellers(force*distance) / MOI 
    % For Yaw: 
    %   tbd
    quadState.ddattitude =  [ ( dot(propThrust,PROP_RELATIVE_POS(:,2)) / IXX ) * RAD_TO_DEG , ...
                              ( dot(propThrust,PROP_RELATIVE_POS(:,1)) / IYY ) * RAD_TO_DEG ,...
                              sum(propTorque) / IZZ * RAD_TO_DEG ]; 
    
    if ADD_NOISE
        quadState.ddattitude = quadState.ddattitude + [ noise(time,1), ...
                                                        noise(time,2), ...
                                                        noise(time,3) ];
    end
                          
    % Solve for velocities and positions using 4th order RK
    %   - Use time steps of DELTA_T_RK for LOOP_TIME; 
    %   - Let Lx be associated with velocities
    %   - Let Kx be associated with positions
    for rungeKuttaLoop = 1:length(0:DELTA_T_RK:LOOP_TIME) 
        % Attitude: 
        L1a = DELTA_T_RK * rkAngAccel(time,quadState.ddattitude); 
        L2a = DELTA_T_RK * rkAngAccel(time + 0.5.*DELTA_T_RK, ...
                                    quadState.ddattitude + 0.5.*L1a);
        L3a = DELTA_T_RK * rkAngAccel(time + 0.5.*DELTA_T_RK, ...
                                    quadState.ddattitude + 0.5.*L2a);
        L4a = DELTA_T_RK * rkAngAccel(time + DELTA_T_RK, ...
                                    quadState.ddattitude + L3a);
        quadState.dattitude = quadState.dattitude + (L1a+2.*L2a+2.*L3a+L4a)./6;

        K1a = DELTA_T_RK * rkAngVelocity(time,quadState.dattitude); 
        K2a = DELTA_T_RK * rkAngVelocity(time + 0.5.*DELTA_T_RK, ...
                                       quadState.dattitude + 0.5.*K1a);
        K3a = DELTA_T_RK * rkAngVelocity(time + 0.5.*DELTA_T_RK, ...
                                       quadState.dattitude + 0.5.*K2a);
        K4a = DELTA_T_RK * rkAngVelocity(time + DELTA_T_RK, ...
                                       quadState.dattitude + K3a);
        quadState.attitude = quadState.attitude + (K1a+2.*K2a+2.*K3a+K4a)./6;

        % Position
        L1t = DELTA_T_RK * rkTransAccel(time,quadState.ddposition); 
        L2t = DELTA_T_RK * rkTransAccel(time + 0.5.*DELTA_T_RK, ...
                                    quadState.ddposition + 0.5.*L1t);
        L3t = DELTA_T_RK * rkTransAccel(time + 0.5.*DELTA_T_RK, ...
                                    quadState.ddposition + 0.5.*L2t);
        L4t = DELTA_T_RK * rkTransAccel(time + DELTA_T_RK, ...
                                    quadState.ddposition + L3t);
        quadState.dposition = quadState.dposition + (L1t+2.*L2t+2.*L3t+L4t)./6;

        K1t = DELTA_T_RK * rkTransVelocity(time,quadState.dposition); 
        K2t = DELTA_T_RK * rkTransVelocity(time + 0.5.*DELTA_T_RK, ...
                                       quadState.dposition + 0.5.*K1t);
        K3t = DELTA_T_RK * rkTransVelocity(time + 0.5.*DELTA_T_RK, ...
                                       quadState.dposition + 0.5.*K2t);
        K4t = DELTA_T_RK * rkTransVelocity(time + DELTA_T_RK, ...
                                       quadState.dposition + K3t);
        quadState.position = quadState.position + (K1t+2.*K2t+2.*K3t+K4t)./6;

        time = time + DELTA_T_RK;
        
        % Add to fine log at every RK update
        if LOG_FINE
            log.fine(j).time = time;
            log.fine(j).roll = quadState.attitude(1);
            log.fine(j).pitch = quadState.attitude(2);
            log.fine(j).yaw = quadState.attitude(3);
            log.fine(j).x = quadState.position(1);
            log.fine(j).y = quadState.position(2);
            log.fine(j).z = quadState.position(3);
            log.fine(j).droll = quadState.dattitude(1);
            log.fine(j).dpitch = quadState.dattitude(2);
            log.fine(j).dyaw = quadState.dattitude(3);
            log.fine(j).dx = quadState.dposition(1);
            log.fine(j).dy = quadState.dposition(2);
            log.fine(j).dz = quadState.dposition(3);
            log.fine(j).ddroll = quadState.ddattitude(1);
            log.fine(j).ddpitch = quadState.ddattitude(2);
            log.fine(j).ddyaw = quadState.ddattitude(3);
            log.fine(j).ddx = quadState.ddposition(1);
            log.fine(j).ddy = quadState.ddposition(2);
            log.fine(j).ddz = quadState.ddposition(3);
            j = j+1;
        end
    end
    
    % Add to coarse log at every PID update
    if LOG_COARSE
        log.coarse(i).time = time;
        log.coarse(i).roll = quadState.attitude(1);
        log.coarse(i).pitch = quadState.attitude(2);
        log.coarse(i).yaw = quadState.attitude(3);
        log.coarse(i).x = quadState.position(1);
        log.coarse(i).y = quadState.position(2);
        log.coarse(i).z = quadState.position(3);
        log.coarse(i).droll = quadState.dattitude(1);
        log.coarse(i).dpitch = quadState.dattitude(2);
        log.coarse(i).dyaw = quadState.dattitude(3);
        log.coarse(i).dx = quadState.dposition(1);
        log.coarse(i).dy = quadState.dposition(2);
        log.coarse(i).dz = quadState.dposition(3);
        log.coarse(i).ddroll = quadState.ddattitude(1);
        log.coarse(i).ddpitch = quadState.ddattitude(2);
        log.coarse(i).ddyaw = quadState.ddattitude(3);
        log.coarse(i).ddx = quadState.ddposition(1);
        log.coarse(i).ddy = quadState.ddposition(2);
        log.coarse(i).ddz = quadState.ddposition(3);
        i = i+1;
    end
end

% -------------------------- Post Processing --------------------------- %

if PLOT_ATTITUDE
    plot_attitude(log,pid);
end

