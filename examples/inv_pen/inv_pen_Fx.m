% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_Fx(x, u)
    m = 1;
    l = 1;
    g = 9.8;
    b = 0;
    I = m*l^2;

    theta = x(1);
    theta_dot = x(2);

    dF_div_dx = zeros(2, 2);

    dF_div_dx(1,1) = 0;
    dF_div_dx(1,2) = 1;
    dF_div_dx(2,1) = -b/I;
    dF_div_dx(2,2) = -m*g*l/I*cos(theta);

    retval = dF_div_dx;

end
