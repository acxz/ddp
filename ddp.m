function [u_traj_opt, traj_cost_opt] = ddp(u_traj)
  
  iters = 5;
  u_traj_opt = rand(size(u_traj,1), size(u_traj,2), iters);
  traj_cost_opt = rand(iters);
  
end