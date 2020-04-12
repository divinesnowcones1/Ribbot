function twist = rbvel2twist(rbvel)
w_hat = rbvel(1:3,1:3); % Upper left 3 by 3 matrix of rbvel
v = rbvel(1:3,4); % Upper right 3 by 1 matrix of rbvel
w = skew2angvel(w_hat);
twist = [v;w];