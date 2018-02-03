close all
clear all
clc

data = load('control.txt');

figure
title('Control')
hold on
plot(data(:,1),'b','DisplayName','X')
plot(data(:,2),'r','DisplayName','Y')
plot(data(:,3),'m','DisplayName','Z')
legend('show')

data2 = load('test9.txt');

figure
title('New')
hold on
plot(data2(:,1),'b','DisplayName','X')
plot(data2(:,2),'r','DisplayName','Y')
plot(data2(:,3),'m','DisplayName','Z')
legend('show')


