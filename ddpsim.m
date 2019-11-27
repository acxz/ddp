function [x_hist, u_hist, traj_cost_hist, time_hist] = ...
 ddpsim(func_term_cost, func_run_cost, func_apply_ctrl, func_F, ...,
 func_state_est, learning_rate, init_state, init_ctrl_seq, time_horizon, ...
 plot_traj, print_verbose, print_short)

  % time stuff
  num_timesteps = size(init_ctrl_seq, 2);
  dt = time_horizon / num_timesteps;
  time = 0;
  time_hist = [time];

  % state history
  state_dim = size(init_state, 1);
  x_hist = init_state;
  xo = init_state;

  % control history
  control_dim = size(init_ctrl_seq, 1);
  du = realmax('double') * ones(control_dim, num_timesteps);
  u_hist = [];

  % trajectory cost history
  traj_cost_hist = [];

  % state trajectory
  x_traj = zeros(state_dim, num_timesteps + 1);

  % control sequence
  u_traj = init_ctrl_seq;

  % plot trajectory in real time
  if(plot_traj)
    state_colors = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; [0.4940 0.1840 0.5560]; [0.4660 0.6740 0.1880]];
    ctrl_colors = [[0.9290 0.6940 0.1250]; [0.3010 0.7450 0.9330]; [0.6350 0.0780 0.1840]];
    traj_cost_color = 'k';

    state_plot = figure(1);
    title('State Value(s)')
    xlabel('Time');
    ylabel('Value');
    for sd = 1:state_dim
      state_animated_lines(sd).animatedline = octaveanimatedline(...
          'Color', state_colors(mod(sd - 1,size(state_colors,1)) + 1,:));
          %'DisplayName', ['State ' num2str(sd)]);
    end
    legend

    % Go ahead and plot the first state
    figure(state_plot)
    hold on
    for sd = 1:state_dim
      addpoints(state_animated_lines(sd).animatedline, time_hist(1), x_hist(sd,1));
    end
    legend
    drawnow

    control_plot = figure(2);
    title('Control Value(s)');
    xlabel('Time');
    ylabel('Value');
    for cd = 1:control_dim
      control_animated_lines(cd).animatedline = octaveanimatedline(...
          'Color', ctrl_colors(mod(cd - 1,size(ctrl_colors,2)) + 1,:));
          %'DisplayName', ['Control ' num2str(cd)]);
    end
    legend

    traj_cost_plot = figure(3);
    title('Trajectory Cost');
    xlabel('Time');
    ylabel('Value');
    traj_cost_line = octaveanimatedline('Color', traj_cost_color);
        %'DisplayName', 'Trajectory Cost');
    legend
  end

  % Compute optimized u_traj
  [u_traj_opt, traj_cost_opt] = ddp(u_traj);
  
  % Choose the last optimized value of ddp
  u_traj = u_traj_opt(:,:,end);
  traj_cost = traj_cost_opt(end);
    
  total_timestep_num = 1;
  while (total_timestep_num <= num_timesteps)
    
    % Get control value for this timestep
    u = u_traj(:,total_timestep_num);
    
    % Apply control and log data
    true_x = func_apply_ctrl(x_hist(:,total_timestep_num), u, dt);
    
    % state estimation after applying control
    xo = func_state_est(true_x);

    % Log state data
    x_hist(:,total_timestep_num+1) = xo;

    % Log control data
    u_hist = [u_hist u];

    % Log trajectory cost data
    traj_cost_hist = [traj_cost_hist traj_cost];

    % Move time forward
    time = time + dt;
    time_hist = [time_hist, time];

    % Real time plotting
    if(plot_traj)

      figure(state_plot)
      hold on
      for sd = 1:state_dim
        addpoints(state_animated_lines(sd).animatedline, time_hist(total_timestep_num+1), x_hist(sd, total_timestep_num+1));
      end
      legend

      figure(control_plot)
      hold on
      for cd = 1:control_dim
        addpoints(control_animated_lines(cd).animatedline, time_hist(total_timestep_num), u_hist(cd, total_timestep_num));
      end
      legend

      figure(traj_cost_plot)
      addpoints(traj_cost_line, time_hist(total_timestep_num), traj_cost_hist(total_timestep_num));
      legend
    drawnow
    end

    total_timestep_num = total_timestep_num + 1;
  end
end
