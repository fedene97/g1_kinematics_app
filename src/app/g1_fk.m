function out = g1_fk(q, varargin)
%G1_FK  Forward kinematics of the Unitree G1: joint angles -> poses of pelvis and TCPs.
%   out = g1_fk(q, 'option', value, ...)
%   q: the 29 joint angles, as one of
%      - a 29x1 vector in the model order (see out.joints.joint)
%      - a struct with the joint names as fields, e.g. q.left_knee_joint = 0.5
%        (joints that are not listed are 0)
%      - [] for all joints at zero
%   Options:
%      'degrees'    true if q is in degrees (default false: radians)
%      'pb'         pelvis position [m] (default: see 'ground')
%      'Rb'         pelvis orientation (3x3), or 'rpy' = [roll pitch yaw] [rad]
%      'ground'     true: lift the pelvis so that the lowest sole point is at
%                   z = 0 (default true if 'pb' is not given)
%      'plot'       true: draw the robot with the TCP frames (default true when
%                   called without an output argument)
%      'capsules'   true: also draw the collision capsules (default false)
%      'print'      true: print the poses as a table (default true when called
%                   without an output argument)
%      'jacobians'  true: also compute the 6x35 Jacobians of the TCPs (default false)
%      'axes'       existing axes to draw into
%   Output (world frame; TCP = tool center point), each one a struct with
%   p (3x1, m), R (3x3), rpy (3x1, rad, roll-pitch-yaw), frame (URDF frame name)
%   and, with 'jacobians', J (6x35, [dp/dt; omega] = J*[v_b; w_b; dq]):
%      out.pelvis, out.torso, out.head, out.left_hand, out.right_hand,
%      out.left_foot, out.right_foot      (foot TCP = centre of the sole)
%      out.joints        table: joint name, angle [rad], angle [deg], x, y, z of
%                        every joint frame
%      out.com           centre of mass (3x1, m)
%      out.collisions    .min  smallest distance between capsules [m] (< 0: collision)
%                        .pairs  colliding pairs (cell of text)
%                        .ground  smallest distance from the ground [m], feet excluded
%                        (NaN if the capsules are not available)
%      out.q, out.pb, out.Rb, out.K   angles [rad], base pose, poses of all bodies
%   Example:
%      g1_fk([])                                    % all joints at zero
%      q = struct('left_knee_joint', 60, 'right_knee_joint', 60, ...
%                 'left_hip_pitch_joint', -30, 'right_hip_pitch_joint', -30, ...
%                 'left_ankle_pitch_joint', -30, 'right_ankle_pitch_joint', -30);
%      out = g1_fk(q, 'degrees', true, 'plot', true);
%      out.right_hand.p                             % right hand position [m]
persistent mod C
if isempty(mod)
    [mod, C] = g1_model();
end
o = struct('degrees', false, 'pb', [], 'Rb', [], 'rpy', [], 'ground', [], ...
           'plot', nargout == 0, 'capsules', false, 'print', nargout == 0, ...
           'jacobians', false, 'axes', []);
for i = 1:2:numel(varargin)
    if ~isfield(o, varargin{i}), error('g1_fk:option', 'Unknown option "%s".', varargin{i}); end
    o.(varargin{i}) = varargin{i + 1};
end
qj = leggi_q(mod, q);
if o.degrees, qj = deg2rad(qj); end
fuori = find(qj < mod.qmin - 1e-9 | qj > mod.qmax + 1e-9);
for k = fuori(:)'
    warning('g1_fk:limits', '%s = %.3f rad is outside the limits [%.3f, %.3f]', ...
            mod.giunti{k}, qj(k), mod.qmin(k), mod.qmax(k));
end
Rb = eye(3);
if ~isempty(o.Rb), Rb = o.Rb; elseif ~isempty(o.rpy), Rb = rpy2rot(o.rpy); end
if isempty(o.ground), o.ground = isempty(o.pb); end
pb = zeros(3, 1); if ~isempty(o.pb), pb = o.pb(:); end
K = cinematica_diretta(mod, Rb, pb, qj);
if o.ground
    zmin = inf;
    for f = find(strcmp({mod.frame.tipo}, 'punto'))
        [~, p] = posa_frame(mod, K, f); zmin = min(zmin, p(3));
    end
    pb(3) = pb(3) - zmin;
    K = cinematica_diretta(mod, Rb, pb, qj);
end

tcp = {'pelvis', mod.nome_base; 'torso', 'torso_link'; 'head', 'head_link';
       'left_hand', 'left_rubber_hand'; 'right_hand', 'right_rubber_hand';
       'left_foot', 'left_sole'; 'right_foot', 'right_sole'};
riserva = struct('left_rubber_hand', 'left_wrist_yaw_link', 'right_rubber_hand', 'right_wrist_yaw_link', ...
                 'head_link', 'torso_link');
for i = 1:size(tcp, 1)
    nf = tcp{i, 2};
    if ~any(strcmp(mod.nomi_frame, nf)) && isfield(riserva, nf), nf = riserva.(nf); end
    [R, p] = posa_frame(mod, K, nf);
    s = struct('frame', nf, 'p', p, 'R', R, 'rpy', rpy_da_rot(R));
    if o.jacobians, s.J = jacobiano_frame(mod, K, nf, 'mondo'); end
    out.(tcp{i, 1}) = s;
end
P0 = cell2mat(K.p(2:end));                          % origins of the joint frames
out.joints = table(mod.giunti(:), qj, rad2deg(qj), P0(1, :)', P0(2, :)', P0(3, :)', ...
                   'VariableNames', {'joint', 'q_rad', 'q_deg', 'x', 'y', 'z'});
out.com = baricentro(mod, K);
out.collisions = struct('min', NaN, 'pairs', {{}}, 'ground', NaN);
if ~isempty(C)
    D = distanze_capsule(C, K);
    out.collisions.min = D.min;
    out.collisions.ground = distanze_terreno(C, K).min;
    col = find(D.d < 0);
    out.collisions.pairs = arrayfun(@(k) sprintf('%s - %s (%.1f mm)', C.capsula(D.coppie(1, k)).nome, ...
        C.capsula(D.coppie(2, k)).nome, 1e3 * D.d(k)), col, 'UniformOutput', false);
end
out.q = qj; out.K = K; out.Rb = Rb; out.pb = pb;

if o.print
    fprintf('\n%-11s %-20s %9s %9s %9s %8s %8s %8s\n', 'TCP', 'frame', 'x [m]', 'y [m]', 'z [m]', ...
            'roll', 'pitch', 'yaw');
    for i = 1:size(tcp, 1)
        s = out.(tcp{i, 1});
        fprintf('%-11s %-20s %9.4f %9.4f %9.4f %7.1f° %7.1f° %7.1f°\n', tcp{i, 1}, s.frame, s.p, rad2deg(s.rpy));
    end
    fprintf('%-11s %-20s %9.4f %9.4f %9.4f\n', 'com', '', out.com);
    if ~isnan(out.collisions.min)
        if isempty(out.collisions.pairs)
            fprintf('no collision (smallest capsule distance %.1f mm, ground %.1f mm)\n', ...
                    1e3 * out.collisions.min, 1e3 * out.collisions.ground);
        else
            fprintf('COLLISION: %s\n', strjoin(out.collisions.pairs, '; '));
        end
    end
end
if o.plot
    if isempty(o.axes), figure('Color', 'w', 'Name', 'G1 forward kinematics'); ax = gca; else, ax = o.axes; end
    disegna_g1(mod, K, out, tcp, ax, C, o.capsules);
end
end

% ======================================================================
function qj = leggi_q(mod, q)
if isempty(q)
    qj = zeros(mod.n, 1);
elseif isstruct(q)
    qj = zeros(mod.n, 1);
    f = fieldnames(q);
    for i = 1:numel(f)
        k = find(strcmp(mod.giunti, f{i}), 1);
        if isempty(k), error('g1_fk:joint', 'Joint "%s" does not exist.', f{i}); end
        qj(k) = q.(f{i});
    end
else
    if numel(q) ~= mod.n, error('g1_fk:q', 'q must have %d elements.', mod.n); end
    qj = q(:);
end
end

function disegna_g1(mod, K, out, tcp, ax, C, con_capsule)
cla(ax); hold(ax, 'on');
disegna_posa(mod, K, ax);
if con_capsule && ~isempty(C)
    D = distanze_capsule(C, K);
    rosse = unique(D.coppie(:, D.d < 0));
    disegna_capsule(C, K, ax, rosse(:)', 0.2);
end
col = lines(size(tcp, 1));
for i = 1:size(tcp, 1)
    s = out.(tcp{i, 1});
    plot3(ax, s.p(1), s.p(2), s.p(3), 'o', 'MarkerSize', 7, 'MarkerFaceColor', col(i, :), ...
          'MarkerEdgeColor', 'k');
    for a = 1:3                                     % TCP frame (x red, y green, z blue)
        e = s.R(:, a) * 0.06; cc = zeros(1, 3); cc(a) = 0.8;
        plot3(ax, s.p(1) + [0 e(1)], s.p(2) + [0 e(2)], s.p(3) + [0 e(3)], '-', 'Color', cc, 'LineWidth', 1.5);
    end
    text(ax, s.p(1), s.p(2), s.p(3) + 0.04, strrep(tcp{i, 1}, '_', ' '), 'FontSize', 8);
end
pc = out.com;
plot3(ax, pc(1), pc(2), pc(3), 'kp', 'MarkerSize', 11, 'MarkerFaceColor', 'y');
plot3(ax, pc(1), pc(2), 0, 'kx', 'MarkerSize', 8);
plot3(ax, [pc(1) pc(1)], [pc(2) pc(2)], [0 pc(3)], 'k:');
[X, Y] = meshgrid(linspace(-0.4, 0.4, 2) + out.pb(1), linspace(-0.4, 0.4, 2) + out.pb(2));
surf(ax, X, Y, zeros(size(X)), 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
axis(ax, 'equal'); grid(ax, 'on'); view(ax, 40, 15);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
title(ax, 'G1 forward kinematics (TCP axes: x red, y green, z blue)');
end
