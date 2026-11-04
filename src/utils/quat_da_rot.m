function q = quat_da_rot(R)
%QUAT_DA_ROT  Quaternione unitario [w; x; y; z] da una matrice di rotazione (Shepperd).
t = trace(R);
if t > 0
    s = 2 * sqrt(1 + t);
    q = [s/4; (R(3,2)-R(2,3))/s; (R(1,3)-R(3,1))/s; (R(2,1)-R(1,2))/s];
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    s = 2 * sqrt(1 + R(1,1) - R(2,2) - R(3,3));
    q = [(R(3,2)-R(2,3))/s; s/4; (R(1,2)+R(2,1))/s; (R(1,3)+R(3,1))/s];
elseif R(2,2) > R(3,3)
    s = 2 * sqrt(1 + R(2,2) - R(1,1) - R(3,3));
    q = [(R(1,3)-R(3,1))/s; (R(1,2)+R(2,1))/s; s/4; (R(2,3)+R(3,2))/s];
else
    s = 2 * sqrt(1 + R(3,3) - R(1,1) - R(2,2));
    q = [(R(2,1)-R(1,2))/s; (R(1,3)+R(3,1))/s; (R(2,3)+R(3,2))/s; s/4];
end
q = q / norm(q);
if q(1) < 0, q = -q; end
end
