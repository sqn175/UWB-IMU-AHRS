%% TODO:
% 1. 仿真旋转角大于90°的情况

%% Start of script
addpath('rot_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables

%% Scenario simulation and plot sensor data

% --------- GroundTruth ---------
[gt_p,gt_v,gt_a,gt_euler,gt_euler_rate,sim_t,sim_freq] = gt4();

% --------- Measurements ---------
[Accelerometer, Gyroscope, time, psi_uwb, t_uwb] = genmeasurements(gt_a, gt_euler_rate, gt_euler, sim_t);
len = length(time);

% Plots
figure;
scatter3(gt_p(:,1), gt_p(:,2), gt_p(:,3),'r.');
hold on;
scatter3(gt_p(1,1), gt_p(1,2), gt_p(1,3), 100, '^r',...
        'MarkerFaceColor','w', 'LineWidth', 2);
scatter3(gt_p(end,1), gt_p(end,2), gt_p(end,3), 100, 'sr',...
        'MarkerFaceColor','w', 'LineWidth', 2);
title('trajectory');
legend('Trajectory', 'Start Point', 'End Point');

figure; 
hold on;
plot(time, gt_euler(:,1)/pi*180, 'r');
plot(time, gt_euler(:,2)/pi*180, 'g');
plot(time, gt_euler(:,3)/pi*180, 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Accelerometer');
hold off;

axis(3) = subplot(3,1,3);
hold on;
plot(time, psi_uwb/pi*180, 'b');
legend('psi\_uwb');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('UWB angle(psi)');
hold off;
linkaxes(axis, 'x');

acc_norm = zeros(len,1);
for i = 1:len 
    acc_norm(i) = norm(Accelerometer(i,:));
end
figure;
plot(time, acc_norm/9.8);
xlabel('Time (s)');
ylabel('Accelerometer (g)')
title('Norm of Accelerometer');

%% Process sensor data through algorithm Madgwick+UWB

AHRS = UWBAHRS('SamplePeriod', 0.01, 'Beta', 0.1);

quaternion = zeros(len, 4);
for t = 1:len
    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), psi_uwb(t));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Process sensor data through algorithm Mohony+UWB
% 
% AHRS = MahonyAHRS('SamplePeriod', 0.01, 'Kp', 0.98, 'Ki', 0.02);
% % AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);
% 
% quaternion = zeros(len, 4);
% for t = 1:len
%     AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), psi_uwb(t));	% gyroscope units must be radians
%     quaternion(t, :) = AHRS.Quaternion;
% end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?0
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler_est = quatern2euler(quaternConj(quaternion));	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure;
subplot(3,1,1);
hold on;
plot(time, gt_euler(:,1)/pi*180);
plot(time, euler_est(:,1)/pi*180);
legend('Ground Truth', 'AHRS Results');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('phi');

subplot(3,1,2);
hold on;
plot(time, gt_euler(:,2)/pi*180);
plot(time, euler_est(:,2)/pi*180);
title('theta');

subplot(3,1,3);
hold on;
plot(time, gt_euler(:,3) /pi*180);
plot(time, euler_est(:,3)/pi*180);
title('psi');

% Evaluation
rms = rms(gt_euler - euler_est);
rms = rms/pi*180;
disp('RMS(deg):');
disp(rms);

%% End of script