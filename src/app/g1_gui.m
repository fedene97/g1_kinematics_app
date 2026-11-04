function g1_gui()
%G1_GUI  Forward-kinematics app for the Unitree G1: one slider per joint.
%   Moving a slider (or typing an angle) updates the robot (CAD meshes,
%   skeleton, TCP frames, centre of mass, collision capsules - choose what to show
%   in the Display tab) and the table with the poses of pelvis, torso, head, hands
%   and feet. The feet are kept on the ground.
%   Buttons: zero pose, bent knees, save / load a pose (.json or .csv), export a
%   CSV for the MuJoCo viewer (python/view_pose.py).
%   The last result is available in the base workspace as g1_pose (see g1_fk).
%   See also g1_gui_ik (inverse kinematics), g1_fk (function).
[mod, C, P] = g1_model();
q = zeros(mod.n, 1);

fig = uifigure('Name', 'Unitree G1 - forward kinematics', 'Position', g1_window_pos(1400, 820));
g = uigridlayout(fig, [2 2]);
g.ColumnWidth = {460, '1x'}; g.RowHeight = {'1x', 250};
tabs = uitabgroup(g); tabs.Layout.Row = [1 2]; tabs.Layout.Column = 1;
ax = uiaxes(g); ax.Layout.Row = 1; ax.Layout.Column = 2;
basso = uigridlayout(g, [1 2]); basso.Layout.Row = 2; basso.Layout.Column = 2;
basso.ColumnWidth = {'1x', 210};
tab = uitable(basso, 'ColumnName', {'TCP', 'x [m]', 'y [m]', 'z [m]', 'roll [°]', 'pitch [°]', 'yaw [°]'});
pul = uigridlayout(basso, [7 1]);
pul.RowHeight = {28, 28, 28, 28, 28, '1x', '1x'}; pul.RowSpacing = 4; pul.Padding = [4 4 4 4];
uibutton(pul, 'Text', 'Zero pose', 'ButtonPushedFcn', @(~, ~) imposta(zeros(mod.n, 1)));
uibutton(pul, 'Text', 'Bent knees', 'ButtonPushedFcn', @(~, ~) imposta(postura_casa(mod)));
uibutton(pul, 'Text', 'Save pose...', 'ButtonPushedFcn', @(~, ~) salva_posa());
uibutton(pul, 'Text', 'Load pose...', 'ButtonPushedFcn', @(~, ~) carica_posa());
uibutton(pul, 'Text', 'Export CSV for MuJoCo', 'ButtonPushedFcn', @(~, ~) esporta());
lab = uilabel(pul, 'Text', '', 'WordWrap', 'on'); lab.Layout.Row = [6 7];

gruppi = {'Left leg', 1:6; 'Right leg', 7:12; 'Waist', 13:15; 'Left arm', 16:22; 'Right arm', 23:29};
cursori = gobjects(1, mod.n); campi = gobjects(1, mod.n); valori = gobjects(1, mod.n);
for gi = 1:size(gruppi, 1)
    t = uitab(tabs, 'Title', gruppi{gi, 1});
    idx = gruppi{gi, 2};
    gl = uigridlayout(t, [2 * numel(idx), 2]);
    gl.RowHeight = repmat({22, 38}, 1, numel(idx));
    gl.ColumnWidth = {'1x', 72};
    for j = 1:numel(idx)
        k = idx(j);
        lim = rad2deg([mod.qmin(k) mod.qmax(k)]);
        valori(k) = uilabel(gl, 'Text', '');
        valori(k).Layout.Row = 2 * j - 1; valori(k).Layout.Column = [1 2];
        cursori(k) = uislider(gl, 'Limits', lim, 'Value', 0, ...
                              'ValueChangingFcn', @(src, ev) muovi(k, ev.Value, false), ...
                              'ValueChangedFcn', @(src, ~) muovi(k, src.Value, false));
        cursori(k).Layout.Row = 2 * j; cursori(k).Layout.Column = 1;
        campi(k) = uieditfield(gl, 'numeric', 'Limits', lim, 'Value', 0, 'ValueDisplayFormat', '%.1f', ...
                               'ValueChangedFcn', @(src, ~) muovi(k, src.Value, true));
        campi(k).Layout.Row = 2 * j; campi(k).Layout.Column = 2;
    end
end
controlli_vista(uitab(tabs, 'Title', 'Display'), @cambia_vista, ax, P.exports);

dlg = uiprogressdlg(fig, 'Title', 'Loading CAD meshes', 'Message', 'Reading STL files...');
V = vista_g1(ax, mod, struct('mesh', true, 'capsule', C, 'barra', dlg));
close(dlg);
out = [];
aggiorna();

    function muovi(k, v, da_campo)
        q(k) = deg2rad(v);
        if da_campo, cursori(k).Value = v; else, campi(k).Value = v; end
        aggiorna();
    end
    function imposta(qn)
        q = qn(:);
        for kk = 1:mod.n
            cursori(kk).Value = rad2deg(q(kk));
            campi(kk).Value = rad2deg(q(kk));
        end
        aggiorna();
    end
    function cambia_vista(vis)
        V = vista_g1_visibilita(V, vis);
    end
    function aggiorna()
        for kk = 1:mod.n
            valori(kk).Text = sprintf('%s: %.1f°', strrep(mod.giunti{kk}, '_joint', ''), rad2deg(q(kk)));
        end
        out = g1_fk(q, 'plot', false, 'print', false);
        vista_g1_aggiorna(V, out.K);
        tab.Data = tabella_tcp(out);
        lab.Text = testo_collisioni(out); lab.FontColor = colore_collisioni(out);
        assignin('base', 'g1_pose', out);
    end
    function salva_posa()
        [f, d] = uiputfile({'*.json', 'JSON pose (*.json)'; '*.csv', 'CSV (*.csv)'}, 'Save pose', ...
                           fullfile(P.exports, 'g1_pose.json'));
        if isequal(f, 0), return; end
        try
            g1_save_pose(fullfile(d, f), mod, q);
            lab.Text = ['Saved ' fullfile(d, f)]; lab.FontColor = [0 0 0];
        catch ME
            uialert(fig, ME.message, 'Could not save the pose');
        end
    end
    function carica_posa()
        [f, d] = uigetfile({'*.json;*.csv', 'Pose files (*.json, *.csv)'}, 'Load pose', P.exports);
        if isequal(f, 0), return; end
        try
            [qn, info] = g1_load_pose(fullfile(d, f), mod);
            imposta(qn);
            lab.Text = sprintf('Loaded %s (%d joints not in the file, %d clipped to their limits)', ...
                               f, numel(info.missing), numel(info.clipped));
            lab.FontColor = [0 0 0];
        catch ME
            uialert(fig, ME.message, 'Could not load the pose');
        end
    end
    function esporta()
        file = fullfile(P.exports, 'pose.csv');
        scrivi_pose_csv(file, mod, [0 1], [out.pb out.pb], cat(3, out.Rb, out.Rb), [out.q out.q]);
        lab.Text = sprintf('Saved %s. View it with: python3 python/view_pose.py', file);
        lab.FontColor = [0 0 0];
    end
end

function dati = tabella_tcp(out)
[chiavi, etichette] = g1_tcp_list();
dati = cell(numel(chiavi) + 1, 7);
for i = 1:numel(chiavi)
    s = out.(chiavi{i});
    dati(i, :) = [etichette(i), num2cell(round(s.p', 4)), num2cell(round(rad2deg(s.rpy'), 1))];
end
dati(end, :) = [{'Centre of mass'}, num2cell(round(out.com', 4)), {[], [], []}];
end

function t = testo_collisioni(out)
if isnan(out.collisions.min)
    t = 'Collision model not available (meshes not found)';
elseif isempty(out.collisions.pairs)
    t = sprintf('No collision (min distance %.0f mm)', 1e3 * out.collisions.min);
else
    t = ['COLLISION: ' strjoin(out.collisions.pairs, '; ')];
end
end

function c = colore_collisioni(out)
if ~isnan(out.collisions.min) && ~isempty(out.collisions.pairs), c = [0.8 0 0]; else, c = [0 0.45 0]; end
end
