function [p,v,a,eul,eul_rate,t,freq] = gt2()

freq = 100;  % Groundtruth simulation frequency in Hz.
sim_step = 1/freq;
start_time = 0;

% First: static case at start up, 0-5s
end_time = 5;
t = (start_time+sim_step: sim_step: end_time)';
p = repmat([0 0 0],length(t),1);
v = repmat([0 0 0],length(t),1);
a = repmat([0 0 0],length(t),1);
eul = repmat([0 0 0],length(t),1);
eul_rate = repmat([0 0 0],length(t),1);

% Second: no acceleration, orientation change with time, 5~60s
end_time = 50;
t_i = (start_time+sim_step : sim_step : end_time)';
p_i = [1.2*t_i, t_i 0.1*t_i];
v_i = repmat([1.2, 1, 0.1], length(t_i),1);
a_i = repmat([0, 0, 0], length(t_i),1);
eul_i = [pi/8*cos(0.2*t_i)-pi/8*cos(0.2*t_i(1)), ...
        pi/12*cos(0.2*t_i)-pi/12*cos(0.2*t_i(1)), ...
        pi/6*cos(0.1*t_i)-pi/6*cos(0.1*t_i(1))];
eul_rate_i = [-0.2*pi/8*sin(0.2*t_i), ...
    -0.2*pi/12*sin(0.2*t_i), ...
    -0.1*pi/6*sin(0.1*t_i)];

t = [t; t(end,:) + t_i];
p = [p; p(end,:) + (v(end,:) + a(end,:).*t_i).*t_i + p_i];
v = [v; v(end,:) + a(end,:).*t_i + v_i];
a = [a; a(end,:) + a_i];
eul = [eul; eul(end,:) + eul_rate(end,:).*t_i + eul_i];
eul_rate = [eul_rate; eul_rate(end,:) + eul_rate_i];


end 