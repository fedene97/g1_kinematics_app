function g1_smoke_test()
%G1_SMOKE_TEST  Quick self-test of the non-graphical parts of the app (a few seconds).
%   Run it with   g1_app test   (or directly, with tests/ on the path).
%   Checks: model built from the URDF, collision capsules, g1_fk, pose files
%   (save / load round trip in .json and .csv), a small inverse-kinematics problem.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(here), 'src', 'app'));
g1_setup();
res = {};
fprintf('G1 kinematics app - self-test\n');

[mod, C] = g1_model();
res = check(res, 'model: 29 joints', mod.n == 29);
res = check(res, 'collision capsules built', ~isempty(C) && numel(C.capsula) > 20);

out = g1_fk([], 'plot', false, 'print', false);
res = check(res, 'g1_fk: feet on the ground', abs(out.left_foot.p(3)) < 0.05 && abs(out.right_foot.p(3)) < 0.05);
res = check(res, 'g1_fk: left hand is on the left (y > 0)', out.left_hand.p(2) > out.right_hand.p(2));
res = check(res, 'g1_fk: pelvis above the feet', out.pelvis.p(3) > out.left_foot.p(3) + 0.3);
res = check(res, 'g1_fk: collision data available', ~isnan(out.collisions.min));
qs = struct('left_knee_joint', 60, 'right_knee_joint', 60);
o2 = g1_fk(qs, 'degrees', true, 'plot', false, 'print', false);
res = check(res, 'g1_fk: bent knees lower the pelvis', o2.pelvis.p(3) < out.pelvis.p(3) - 0.05);

% pose files: save / load round trip
q = postura_casa(mod);
for ext = {'.json', '.csv'}
    f = fullfile(tempdir, ['g1_smoke_pose' ext{1}]);
    g1_save_pose(f, mod, q);
    [q2, info] = g1_load_pose(f, mod);
    res = check(res, ['pose file ' ext{1} ': round trip'], max(abs(q2 - q)) < 1e-5 && isempty(info.missing));
    delete(f);
end

% inverse kinematics: reach the hand position of a known configuration, pelvis fixed
arm = 16:22;
qt = postura_casa(mod); qt(arm(1)) = qt(arm(1)) - 0.4; qt(arm(4)) = qt(arm(4)) + 0.3;
Kt = cinematica_diretta(mod, eye(3), zeros(3, 1), qt);
[~, pt] = posa_frame(mod, Kt, 'left_wrist_yaw_link');
x0 = stato_cin(mod, 'qj', postura_casa(mod));
comp = compito('frame', 'nome', 'left_wrist_yaw_link', 'p', pt);
[x1, ik] = ik_risolvi(mod, x0, comp, struct('flottante', false, 'giunti', arm, 'riavvii', 3));
K1 = cinematica_diretta(mod, eye(3), zeros(3, 1), x1.qj);
[~, p1] = posa_frame(mod, K1, 'left_wrist_yaw_link');
res = check(res, 'inverse kinematics: hand reaches the target', ik.convergenza && norm(p1 - pt) < 1e-4);

ok = all(cellfun(@(r) r{2}, res));
if ok
    fprintf('All %d checks passed.\n', numel(res));
else
    error('g1_smoke_test:failed', 'Some checks failed (see above).');
end
end

function res = check(res, name, ok)
res{end + 1} = {name, logical(ok)};
if ok, s = 'ok  '; else, s = 'FAIL'; end
fprintf('  [%s] %s\n', s, name);
end
