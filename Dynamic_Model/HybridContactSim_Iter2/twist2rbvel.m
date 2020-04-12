function rbvel = twist2rbvel(twist)
v = twist(1:3);
w = twist(4:6);
rbvel = [angvel2skew(w) v; zeros(1,4)];