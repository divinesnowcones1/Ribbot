function Ad_g = tform2adjoint(g)
R = g(1:3,1:3); % Upper left 3 by 3 matrix of g
p = g(1:3,4); % Upper right 3 by 1 matrix of g
p_hat = angvel2skew(p);
Ad_g = [R p_hat*R; zeros(3,3) R];