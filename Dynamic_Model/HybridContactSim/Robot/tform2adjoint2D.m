function Adg = tform2adjoint2D(g)
%tform2adjoint Maps the rigid body transformation g, in homogeneous coor-
%              dinates, to the transformation adjoint matrix, Adg
R = [g(1,1) g(1,2);g(2,1) g(2,2)];
p_hat = [g(2,3);-g(1,3)];
Adg = [R p_hat;zeros(1,2) 1];
end

