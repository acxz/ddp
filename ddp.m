function [opt_u_trajs, opt_traj_costs] = ddp(func_control_update_converged, ...
      func_term_cost, func_run_cost, func_F, func_Fx, func_Fu, learning_rate, init_state, u_traj, time_horizon)
  
  % time stuff
  num_timesteps = size(u_traj, 2);
  dt = time_horizon / num_timesteps;
  
  % control stuff
  control_dim = size(u_traj, 1);
  du = realmax('double') * ones(control_dim, num_timesteps);
  opt_u_traj = u_traj;
  opt_u_trajs = [opt_u_traj];
  
  % state stuff
  state_dim = size(init_state, 1);
  x_traj = zeros(state_dim, num_timesteps+1);
  x_traj(:,1) = init_state;
  
  % create initial state trajectory prediction and trajectory cost
  first_traj_cost = 0;
  for timestep_num = 1:num_timesteps
    x_traj(:,timestep_num+1) = func_F(x_traj(:,timestep_num), u_traj(:,timestep_num), dt);
    first_traj_cost = first_traj_cost + func_run_cost(x_traj(:,timestep_num), u_traj(:,timestep_num));
  end
  first_traj_cost = first_traj_cost + func_term_cost(x_traj(:,end));
  
  % cost stuff
  opt_traj_costs = [first_traj_cost];
  
  iteration = 1;
  while(func_control_update_converged(du, iteration) == false)
  
    % Calculate dynamics and cost gradients/hessians
    dfxi = zeros(state_dim, state_dim, num_timesteps);
    dfu = zeros(state_dim, control_dim, num_timesteps);
    for timestep_num = 1:num_timesteps
      dfxi(:,:,timestep_num) = eye(state_dim, state_dim) + func_Fx(x_traj(:,timestep_num), opt_u_traj(:,timestep_num));
      dfu(:,:,timestep_num) = func_Fu(x_traj(:,timestep_num), opt_u_traj(:,timestep_num));
    end
    
    
    
    
    Qx = zeros(state_dim, num_timesteps);
    Qu = zeros(control_dim, num_timesteps);
    Qxu = zeros(state_dim, control_dim, num_timesteps);
    Qux = zeros(control_dim, state_dim, num_timesteps);
    Qxx = zeros(state_dim, state_dim, num_timesteps);
    Quu = zeros(control_dim, control_dim, num_timesteps);
    
    % TODO: Calculate feed_forward and feed_back terms
    feed_forward_term = zeros(control_dim, num_timesteps);
    feed_back_term = zeros(control_dim, state_dim, num_timesteps);
    for timestep_num = 1:num_timesteps
      feed_forward_term(:,timestep_num) = -1 * Quu(:,:,timestep_num) \ Qu(:,timestep_num);
      feed_back_term(:,:,timestep_num) = -1 * Quu(:,:,timestep_num) \ Qxu(:,timestep_num);
    end
    
    % Calcuate optimal control correction
    du = zeros(control_dim, num_timesteps);
    dx = zeros(state_dim, num_timesteps);
    for timestep_num = 1:num_timesteps
      du(:,timestep_num) = feed_forward_term(:,timestep_num) + feed_back_term(:,:,timestep_num) * dx(:,timestep_num);
      dx(:,timestep_num+1) = dfxi(:,:,timestep_num) * dx(:,timestep_num) + dfu(:,:,timestep_num) * du(:,timestep_num);
    end
    
    % Calculate optimized trajectory
    opt_u_traj = opt_u_traj + learning_rate * du;
      
    % Calculate trajectory cost of optimized trajectory
    x_traj = zeros(state_dim, num_timesteps+1);
    x_traj(:,1) = init_state;
    opt_traj_cost = 0;
    for timestep_num = 1:num_timesteps
      x_traj(:,timestep_num+1) = func_F(x_traj(:,timestep_num), opt_u_traj(:,timestep_num), dt);
      opt_traj_cost = opt_traj_cost + func_run_cost(x_traj(:,timestep_num), opt_u_traj(:,timestep_num));
    end
    opt_traj_cost = opt_traj_cost + func_term_cost(x_traj(:,end));
      
    % Add optimized trajectory to the output
    opt_u_trajs = cat(3, opt_u_trajs, opt_u_traj);
      
    % Add trajectory cost of optimized trajectory to the output
    opt_traj_costs = [opt_traj_costs; opt_traj_cost];
      
    fprintf("IN: %d, DU: %d\n", iteration, mean(sum(abs(du),1)));
    iteration = iteration + 1;
  end
  
end