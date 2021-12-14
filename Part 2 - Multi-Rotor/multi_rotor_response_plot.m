%% Plots the postion response data saved in position_responses.mat
clear all
clc

load('position_responses.mat')
x = positions.signals.values(:, 1);
y = positions.signals.values(:, 2);
z = positions.signals.values(:, 3);
x_ref = positions.signals.values(:, 4);
y_ref = positions.signals.values(:, 5);
z_ref = positions.signals.values(:, 6);
t = positions.time;

figure(1);
set(gcf,'color','none');
hold on

plot(t, x_ref, '--', 'LineWidth', 2)
plot(t, x, '-', 'LineWidth', 2)
plot(t, y_ref, '--', 'LineWidth', 2)
plot(t, y, '-', 'LineWidth', 2)
plot(t, z_ref, '--', 'LineWidth', 2)
plot(t, z, '-', 'LineWidth', 2)

ylim([-7, 7])
xlabel('Time (s)')
ylabel('Position (m)')
legend('X Input', 'X Response', 'Y Input', 'Y Response', 'Z Input', 'Z Response')
title('Multi-Rotor sUAS Position Response')


