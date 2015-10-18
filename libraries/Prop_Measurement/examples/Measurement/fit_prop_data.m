% Tao Du
% taodu@csail.mit.edu
% Octo 18, 2015
%
% Linear least square fitting.

clear all; clc; close all;
data_file_name = 'prop2.txt';

% Data file format: pwm in the first column and thrust in the second
% column.
D = dlmread(data_file_name);
pwm = D(:, 1);
thrust = D(:, 2);

% Linear least square fitting(only starts from the first nonzero thrust).
non_zero_index = find(thrust > 0);
non_zero_pwm = pwm(non_zero_index);
non_zero_thrust = thrust(non_zero_index);
% Linear regression: Y = XB.
Y = non_zero_thrust;
X = [non_zero_pwm ones(size(non_zero_pwm))];
B = X \ Y;

% Plot the result.
figure;
% Plot the data points
plot(pwm, thrust, 'r*');
hold on;
% Plot the fitted line.
plot(non_zero_pwm, non_zero_pwm * B(1) + B(2), 'b');
% Add axis range, title, etc.
axis([min(pwm), max(pwm) 0, max(thrust)]);
xlabel('pwm');
ylabel('thrust: g');
title(['pwm vs thrust(data from ', data_file_name]);