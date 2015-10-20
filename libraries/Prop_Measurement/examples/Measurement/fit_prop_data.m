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

% Fit the thrust vs pwm profile. Normalize them for stability.
% thrust / 1000 = a * (pwm / 1000 - b)^2.
% thrust = 1000a * (pwm / 1000 - b)^2.
normalized_pwm = pwm / 1000;
normalized_thrust = thrust / 1000;
p = quad_fit(normalized_pwm, normalized_thrust);

% Plot the result.
figure;
% Plot the data points
plot(pwm, thrust, 'r*');
hold on;
% Plot the fitted line.
plot(pwm, p(pwm / 1000) * 1000, 'b');
% Add axis range, title, etc.
axis([min(pwm), max(pwm) 0, max(thrust)]);
xlabel('pwm');
ylabel('thrust: g');
title(['pwm vs thrust(data from ', data_file_name]);