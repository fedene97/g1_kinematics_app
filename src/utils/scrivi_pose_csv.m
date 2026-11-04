function scrivi_pose_csv(file, mod, t, pb, Rb, Q)
%SCRIVI_POSE_CSV  Scrive una sequenza di pose per la visualizzazione in MuJoCo.
%   t: 1xN tempi; pb: 3xN; Rb: 3x3xN; Q: n x N angoli dei giunti (ordine del modello).
%   Colonne: t, pb_x, pb_y, pb_z, qw, qx, qy, qz, poi i giunti con il loro nome.
N = numel(t);
fid = fopen(file, 'w');
c = onCleanup(@() fclose(fid));
fprintf(fid, 't,pb_x,pb_y,pb_z,qw,qx,qy,qz');
fprintf(fid, ',%s', mod.giunti{:});
fprintf(fid, '\n');
for s = 1:N
    q = quat_da_rot(Rb(:, :, s));
    fprintf(fid, '%.6f', t(s));
    fprintf(fid, ',%.9f', pb(:, s));
    fprintf(fid, ',%.9f', q);
    fprintf(fid, ',%.9f', Q(:, s));
    fprintf(fid, '\n');
end
fprintf('  pose scritte: %s (%d campioni)\n', file, N);
end
