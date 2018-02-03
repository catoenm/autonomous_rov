% ---------------------------------------------------------------------- %
%
% Quadcopter Simulator - fitCurves.m
% Script to fit curves to motor/prop data
% 
% Author: Devon Copeland
% 
% © Cyclone Robotics 2016
%
% Notes: 
%
%
% ---------------------------------------------------------------------- %

clc;
clear all; 
close all; 

GRAMS_TO_NEWTONS = 0.00980665;

load('TestData/TestData_CF9x3_Quanum1000KV2212MT');

RPM_TO_DRAG = polyfit(rpm4,drag4,2);
THROTTLE_TO_RPM = polyfit(throttle1,rpm1,2);
RPM_TO_THRUST = polyfit(rpm1,thrust1*GRAMS_TO_NEWTONS,2);

figure; 
hold on; 
plot(throttle2,thrust2);
plot(throttle3,thrust3);
plot(throttle2, polyval(RPM_TO_THRUST, polyval(THROTTLE_TO_RPM,throttle2))/GRAMS_TO_NEWTONS);

figure;
hold on;
plot(rpm4,drag4);
plot(rpm4, polyval(RPM_TO_DRAG,rpm4));
