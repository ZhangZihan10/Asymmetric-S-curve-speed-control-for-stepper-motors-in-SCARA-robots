clear; clc;

% Multi-waypoint asymmetric S-curve demo
waypoints = [0; 1200; 2200; 1800; 3000]; % step

cfg.Ts = 0.001;
cfg.v_max = 1800;   % step/s
cfg.a_acc = 4500;   % step/s^2
cfg.a_dec = 6500;   % step/s^2 (asymmetric)
cfg.j_acc = 25000;  % step/s^3
cfg.j_dec = 38000;  % step/s^3 (asymmetric)
cfg.v_start = 0;
cfg.v_end = 0;

traj = plan_multiwaypoint_asym_scurve(waypoints, cfg);

% For Simulink From Workspace block
vel_ref_ts = timeseries(traj.vel, traj.t);
pos_ref_ts = timeseries(traj.pos, traj.t);

figure('Name','Asymmetric S-Curve Multi-Waypoint');
subplot(3,1,1);
plot(traj.t, traj.pos, 'LineWidth', 1.4); grid on;
ylabel('Position');

subplot(3,1,2);
plot(traj.t, traj.vel, 'LineWidth', 1.4); grid on;
ylabel('Velocity');

subplot(3,1,3);
plot(traj.t, traj.acc, 'LineWidth', 1.2); hold on;
plot(traj.t, traj.jerk, 'LineWidth', 1.0);
grid on;
ylabel('Acc/Jerk'); xlabel('Time (s)');
legend('acc','jerk');

fprintf('Trajectory samples: %d, total time: %.3f s\n', numel(traj.t), traj.t(end));
