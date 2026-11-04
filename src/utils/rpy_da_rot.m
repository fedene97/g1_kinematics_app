function rpy = rpy_da_rot(R)
%RPY_DA_ROT  Angoli roll-pitch-yaw (convenzione URDF) da una matrice di rotazione.
r = atan2(R(3,2), R(3,3));
p = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
y = atan2(R(2,1), R(1,1));
rpy = [r; p; y];
end
