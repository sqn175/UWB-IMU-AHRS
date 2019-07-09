function [acc_m, gyro_m, t_imu, psi_uwb, t_uwb] = genmeasurements(acc, eul_rate, eul, sim_t)

step = sim_t(2)-sim_t(1);
sim_freq = 1/step; 
imu_freq = 100;   % IMU measurements frequency in Hz
sim_len = length(sim_t);

g = [0 0 -9.8]; % Gravity
% Noise-free IMU measurements
acc_true = zeros(sim_len,3);
gyro_true = zeros(sim_len,3);
for i = 1:sim_len
    C_nb = euler2rotMat(eul(i,1),eul(i,2),eul(i,3)); % sensor to earth frame
    acc_true(i,:) = C_nb' * (acc(i,:) - g)';
    
    J = euler2jm(eul(i,1),eul(i,2),eul(i,3));
    gyro_true(i,:) = J * eul_rate(i,:)';
end

% Generate white Gaussion noise 
rng(1);% Generate random numbers that are repeatable
% sigma_acc = 2e-3;
% sigma_b_acc = 8e-4;
% sigma_gyro = 6e-4; 
% sigma_b_gyro = 2e-4;
sigma_acc = 2e-2;
sigma_b_acc = 8e-3;
sigma_gyro = 2e-3; 
sigma_b_gyro = 2e-3;

% b_a_init = [0.02;-0.3;-0.1];
% b_g_init = [0.01;-0.06;0.02];
b_a_init = [0,0,0];
b_g_init = [0,0,0];
acc_noise  = random('Normal', 0, sigma_acc, sim_len, 3);
gyro_noise = random('Normal', 0, sigma_gyro, sim_len, 3);
b_a_noise = random('Normal', 0, sigma_b_acc, sim_len, 3);
b_g_noise = random('Normal', 0, sigma_b_gyro, sim_len, 3);
b_a = zeros(sim_len,3);
b_g = zeros(sim_len,3);
b_a(1,:) = b_a_init;
b_g(1,:) = b_g_init;
for i = 2:sim_len
    rate_a = (b_a_noise(i,:) + b_a_noise(i-1,:)) / 2;
    b_a(i,:) = b_a(i-1,:) + step*rate_a;

    rate_g = (b_g_noise(i,:) + b_g_noise(i-1,:)) / 2;
    b_g(i,:) = b_g(i-1,:) + step*rate_g;
end

% We obatain the 100Hz IMU measurements corrupted with noise and bias
sample = 1: sim_freq / imu_freq: sim_len;
acc_m = acc_true(sample,:) + b_a(sample,:) + acc_noise(sample,:);
gyro_m = gyro_true(sample,:) + b_g(sample,:) + gyro_noise(sample,:);

t_imu = sim_t(sample);  % IMU timestamp

% --------- UWB angles measurements ---------
% Currently 100Hz
% add noise to psi_uwb
psi_uwb = eul(:,3);
uwb_angle_sigma = 5 /180*pi; % in rad
uwb_angle_noise = random('Normal', 0, uwb_angle_sigma, length(psi_uwb),1);
psi_uwb = psi_uwb + uwb_angle_noise;

t_uwb = t_imu; 
end