% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - plot_attitude.m
% Function to plot the attitude of the quad as a function of time
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
%
% Notes:
%
% ---------------------------------------------------------------------- %

function plot_attitude(log,pid)
    % Plot roll:
    figure;
    % Angle
    subplot(3,1,1);
    plot([log.fine.time],[log.fine.roll]);
    title(sprintf('Roll vs Time\nPID values:  P = %.1d,  I = %.1d,  D = %.1d\n', pid.p(1), pid.i(1), pid.d(1)));
    xlabel('time (s)');
    ylabel('\theta (deg)');
    % Angular velocity
    subplot(3,1,2);
    plot([log.fine.time],[log.fine.droll]);
    xlabel('time (s)');
    ylabel('\omega (deg/s)');
    % Angular acceleration
    subplot(3,1,3);
    plot([log.fine.time],[log.fine.ddroll]);
    xlabel('time (s)');
    ylabel('\alpha (deg/s^2)');

    % Plot pitch:
    figure;
    % Angle
    subplot(3,1,1);
    plot([log.fine.time],[log.fine.pitch]);
    title(sprintf('Pitch vs Time\nPID values:  P = %.1d,  I = %.1d,  D = %.1d\n', pid.p(2), pid.i(2), pid.d(2)));
    xlabel('time (s)');
    ylabel('\theta (deg)');
    % Angular velocity
    subplot(3,1,2);
    plot([log.fine.time],[log.fine.dpitch]);
    xlabel('time (s)');
    ylabel('\omega (deg/s)');
    % Angular acceleration
    subplot(3,1,3);
    plot([log.fine.time],[log.fine.ddpitch]);
    xlabel('time (s)');
    ylabel('\alpha (deg/s^2)');

    % Plot yaw:
    figure;
    % Angle
    subplot(3,1,1);
    plot([log.fine.time],[log.fine.yaw]);
    title(sprintf('Yaw vs Time\nPID values:  P = %.1d,  I = %.1d,  D = %.1d\n', pid.p(3), pid.i(3), pid.d(3)));
    xlabel('time (s)');
    ylabel('\theta (deg)');
    % Angular velocity
    subplot(3,1,2);
    plot([log.fine.time],[log.fine.dyaw]);
    xlabel('time (s)');
    ylabel('\omega (deg/s)');
    % Angular acceleration
    subplot(3,1,3);
    plot([log.fine.time],[log.fine.ddyaw]);
    xlabel('time (s)');
    ylabel('\alpha (deg/s^2)');
end