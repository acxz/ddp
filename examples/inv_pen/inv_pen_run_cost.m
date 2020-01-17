% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_run_cost(x, u)

    Q = [1, 0; 0, 1];
    goal_state = [pi; 0];
    
    R = [0.001];

    retval = 1/2 * sum((x - goal_state) .* (Q * (x - goal_state)),1) + ...
             1/2 * sum((u .* (R * u)), 1);

end
