% Author: Akash Patel (apatel435)
% Date: 5/29/19

function retval = inv_pen_Fu(x, u)
    m = 1;
    l = 1;
    g = 9.8;
    b = 0;
    I = m*l^2;

    theta = x(1,:);
    theta_dot = x(2,:);

    dF_div_du = zeros(2,1);

    dF_div_du(1,1) = 0;
    dF_div_du(2,1) = 1/I;
    
    retval = dF_div_du;

end
