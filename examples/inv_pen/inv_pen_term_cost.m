% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_term_cost(x)

    Qf = [1000, 0; 0, 10];
    goal_state = [pi; 0];

    retval = 1/2 * sum((x - goal_state) .* (Qf * (x - goal_state)),1);

end
