function [p,v,a,eul,eul_rate,sim_t,freq] = gt1()

freq = 100;  % Groundtruth simulation frequency in Hz.
sim_step = 1/freq;
start_time = 0;
% Four situations
% First: static case at start up, 0-5s
end_time = 100;
sim_t = (start_time: sim_step: end_time)';
p = [3*sim_t, sim_t, sim_t]; % Position in inertial frame
v = repmat([3, 1, 1],length(sim_t),1); % velocity, derivative of position
a = repmat([0, 0, 0],length(sim_t),1); % accelation, derivative of velocity

euler_amp = 0.2;
eul = [euler_amp*sin(0.1*sim_t), 2*euler_amp*sin(0.1*sim_t), 4*euler_amp*sin(0.1*sim_t)];  % Euler angles in inertial frame
eul_rate = [0.1*euler_amp*cos(0.1*sim_t), 0.2*euler_amp*cos(0.1*sim_t), 0.4*euler_amp*cos(0.1*sim_t)]; % the rate of change of the Euler angles
end