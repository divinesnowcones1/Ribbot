function angvel = skew2angvel(skew)
a1 = skew(3,2);
a2 = skew(1,3);
a3 = skew(2,1);
angvel = [a1;a2;a3];
end