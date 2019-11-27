disp("DPP implementation on inv_pen in Octave/MATLAB")

time_horizon = 1; % in seconds
num_timesteps = 10;
ctrl_dim = 1;
init_ctrl_seq = randn(ctrl_dim, num_timesteps);
init_state = [0; 0];
learning_rate = 0.01;
plot_traj = true;
print_verbose = true;
print_short = false;
addpath(genpath('../..'));

[x_hist, u_hist, traj_cost_hist, time_hist] = ddpsim(@inv_pen_term_cost, ...
 @inv_pen_run_cost, @inv_pen_apply_ctrl, @inv_pen_F, @inv_pen_state_est, learning_rate, ...,
 init_state, init_ctrl_seq, time_horizon, plot_traj, print_verbose, ...
 print_short);

all_figures = findobj('type', 'figure');
num_figures = length(all_figures);

figure(num_figures + 1);
hold on;
title('State');
xlabel('Time (s)');
ylabel('Value');
plot(time_hist, x_hist(1,:));
plot(time_hist, x_hist(2,:));
legend('theta', 'theta\_dot');

figure(num_figures + 2);
hold on;
title('Control');
xlabel('Time (s)');
ylabel('Value');
plot(time_hist, [u_hist(1,:), 0]);
legend('Torque');

figure(num_figures + 3);
hold on;
title('Trajectory Cost');
xlabel('Time (s)');
ylabel('Cost');
plot(time_hist, [rep_traj_cost_hist, 0]);
legend('Cost');

disp("Finished")