function g1_gui_ik()
%G1_GUI_IK  Inverse-kinematics app for the Unitree G1: move the TCPs, read the joints.
%   Pick a TCP (tool center point: hands, feet, pelvis, torso), move it with the
%   sliders (position and orientation relative to its starting pose) or type the
%   values; the inverse kinematics (ik_risolvi) computes the 29 joint angles, shown
%   in the table on the right with their limits (red row = joint at its limit).
%   Modes:
%     - Feet on ground: floating base, feet fixed on the ground (priority 1), then
%       the centre of mass over the feet (optional), then the active TCPs, then
%       the posture
%     - Fixed pelvis: pelvis fixed, active TCPs, then the posture
%   Every TCP can be active (follows its target) or free; for the hands the
%   orientation can be controlled or not. "Avoid collisions" adds the capsule
%   collision task at priority 1. Poses can be saved / loaded (.json or .csv, the
%   29 joint angles) and exported as a CSV for the MuJoCo viewer (python/view_pose.py).
%   The last solution is available in the base workspace as g1_ik (x, q, joints).
%   See also g1_gui (forward kinematics), g1_fk (function).
[mod, C, P] = g1_model();

chiavi = {'left_hand', 'right_hand', 'left_foot', 'right_foot', 'pelvis', 'torso'};
etichette_tcp = {'Left hand', 'Right hand', 'Left foot', 'Right foot', 'Pelvis', 'Torso'};
out0 = g1_fk([], 'plot', false, 'print', false);
frame = cellfun(@(c) out0.(c).frame, chiavi, 'UniformOutput', false);
x = []; T = []; sel = 1; p0 = []; R0 = []; ultimo = [];

fig = uifigure('Name', 'Unitree G1 - inverse kinematics', 'Position', g1_window_pos(1450, 840));
g = uigridlayout(fig, [2 3]);
g.ColumnWidth = {400, '1x', 330}; g.RowHeight = {'1x', 150};
tabs = uitabgroup(g); tabs.Layout.Row = [1 2]; tabs.Layout.Column = 1;
ax = uiaxes(g); ax.Layout.Row = 1; ax.Layout.Column = 2;
tg = uitable(g, 'ColumnName', {'Joint', 'q [°]', 'min [°]', 'max [°]'}, 'ColumnWidth', {150, 55, 55, 55});
tg.Layout.Row = [1 2]; tg.Layout.Column = 3;
stato = uitextarea(g, 'Editable', 'off'); stato.Layout.Row = 2; stato.Layout.Column = 2;

t1 = uitab(tabs, 'Title', 'Targets');
gl = uigridlayout(t1, [18 2]);
gl.RowHeight = [repmat({26}, 1, 6), repmat({22, 34}, 1, 6)];
gl.ColumnWidth = {'1x', '1x'};
uilabel(gl, 'Text', 'Mode');
dd_modo = uidropdown(gl, 'Items', {'Feet on ground', 'Fixed pelvis'}, 'ValueChangedFcn', @(~, ~) riparti());
uilabel(gl, 'Text', 'TCP to move');
dd_tcp = uidropdown(gl, 'Items', etichette_tcp, 'ItemsData', chiavi, 'ValueChangedFcn', @(~, ~) seleziona());
cb_att = uicheckbox(gl, 'Text', 'Active (follows target)', 'ValueChangedFcn', @(~, ~) opzioni_tcp());
cb_ori = uicheckbox(gl, 'Text', 'Control orientation', 'ValueChangedFcn', @(~, ~) opzioni_tcp());
cb_col = uicheckbox(gl, 'Text', 'Avoid collisions', 'Value', false, 'ValueChangedFcn', @(~, ~) risolvi());
cb_com = uicheckbox(gl, 'Text', 'CoM over feet', 'Value', true, 'ValueChangedFcn', @(~, ~) risolvi());
uibutton(gl, 'Text', 'Reset pose', 'ButtonPushedFcn', @(~, ~) riparti());
uibutton(gl, 'Text', 'Export CSV for MuJoCo', 'ButtonPushedFcn', @(~, ~) esporta());
uibutton(gl, 'Text', 'Save pose...', 'ButtonPushedFcn', @(~, ~) salva_posa());
uibutton(gl, 'Text', 'Load pose...', 'ButtonPushedFcn', @(~, ~) carica_posa());
etichette = {'dx [m]', 'dy [m]', 'dz [m]', 'droll [°]', 'dpitch [°]', 'dyaw [°]'};
limiti = {[-0.4 0.4], [-0.4 0.4], [-0.4 0.4], [-90 90], [-90 90], [-90 90]};
lab_c = gobjects(1, 6); cur = gobjects(1, 6); ed = gobjects(1, 6);
for i = 1:6
    lab_c(i) = uilabel(gl, 'Text', etichette{i});
    lab_c(i).Layout.Row = 6 + 2 * i - 1; lab_c(i).Layout.Column = [1 2];
    sub = uigridlayout(gl, [1 2]);
    sub.Layout.Row = 6 + 2 * i; sub.Layout.Column = [1 2];
    sub.ColumnWidth = {'1x', 72}; sub.Padding = [0 0 0 0];
    cur(i) = uislider(sub, 'Limits', limiti{i}, 'Value', 0, ...
                      'ValueChangingFcn', @(src, ev) muovi(i, ev.Value), ...
                      'ValueChangedFcn', @(src, ~) muovi(i, src.Value));
    ed(i) = uieditfield(sub, 'numeric', 'Limits', limiti{i}, 'Value', 0, 'ValueDisplayFormat', '%.3g', ...
                        'ValueChangedFcn', @(src, ~) muovi(i, src.Value));
end
controlli_vista(uitab(tabs, 'Title', 'Display'), @cambia_vista, ax, P.exports);

dlg = uiprogressdlg(fig, 'Title', 'Loading CAD meshes', 'Message', 'Reading STL files...');
V = vista_g1(ax, mod, struct('mesh', true, 'capsule', C, 'barra', dlg));
close(dlg);
riparti();

    % ------------------------------------------------------------------
    function riparti(q0)
        % starting pose (bent knees unless q0 is given), feet on the ground; targets = current poses
        if nargin < 1 || isempty(q0), q0 = postura_casa(mod); end
        out = g1_fk(q0, 'plot', false, 'print', false);
        x = stato_cin(mod, 'pb', out.pb, 'Rb', out.Rb, 'qj', out.q);
        K = cinematica_diretta(mod, x.Rb, x.pb, x.qj);
        flott = strcmp(dd_modo.Value, 'Feet on ground');
        T = struct('p', {}, 'R', {}, 'attivo', {}, 'orient', {});
        for k = 1:numel(chiavi)
            [R, p] = posa_frame(mod, K, frame{k});
            piede = endsWith(chiavi{k}, 'foot');
            mano = endsWith(chiavi{k}, 'hand');
            T(k) = struct('p', p, 'R', R, 'attivo', piede || mano, 'orient', piede || ~mano);
            if ~flott && strcmp(chiavi{k}, 'pelvis'), T(k).attivo = false; end
        end
        cb_com.Enable = onoff(flott);
        seleziona();
    end
    function seleziona()
        sel = find(strcmp(chiavi, dd_tcp.Value));
        p0 = T(sel).p; R0 = T(sel).R;
        for i2 = 1:6
            cur(i2).Value = 0; ed(i2).Value = 0;
        end
        cb_att.Value = T(sel).attivo; cb_ori.Value = T(sel).orient;
        flott = strcmp(dd_modo.Value, 'Feet on ground');
        bloccato = flott && endsWith(chiavi{sel}, 'foot');         % feet on the ground are always constrained
        cb_att.Enable = onoff(~bloccato && ~(~flott && strcmp(chiavi{sel}, 'pelvis')));
        risolvi();
    end
    function opzioni_tcp()
        T(sel).attivo = cb_att.Value; T(sel).orient = cb_ori.Value;
        risolvi();
    end
    function muovi(i2, v)
        cur(i2).Value = v; ed(i2).Value = v;
        d = [cur.Value];
        T(sel).p = p0 + d(1:3)';
        T(sel).R = rpy2rot(deg2rad(d(4:6))) * R0;
        if ~T(sel).attivo, T(sel).attivo = true; cb_att.Value = true; end
        risolvi();
    end
    function cambia_vista(vis)
        V = vista_g1_visibilita(V, vis);
    end
    function risolvi()
        flott = strcmp(dd_modo.Value, 'Feet on ground');
        comp = compito('postura', 'q', postura_casa(mod), 'peso', 0.02, 'priorita', 5);
        if cb_col.Value && ~isempty(C)
            comp(end + 1) = compito('collisioni', 'modello', C, 'distanza', 0.01, 'priorita', 1);
        end
        for k = 1:numel(chiavi)
            piede = endsWith(chiavi{k}, 'foot');
            if flott && piede
                pr = 1; att = true;
            else
                pr = 3; att = T(k).attivo;
            end
            if ~flott && strcmp(chiavi{k}, 'pelvis'), att = false; end
            if ~att, continue; end
            if T(k).orient
                comp(end + 1) = compito('frame', 'nome', frame{k}, 'p', T(k).p, 'R', T(k).R, 'priorita', pr); %#ok<AGROW>
            else
                comp(end + 1) = compito('frame', 'nome', frame{k}, 'p', T(k).p, 'priorita', pr); %#ok<AGROW>
            end
        end
        if flott && cb_com.Value
            ps = (T(3).p + T(4).p) / 2;                            % T(3), T(4): the feet
            comp(end + 1) = compito('com', 'p', [ps(1:2); 0], 'maschera', [1 1 0], 'priorita', 2);
        end
        opz = struct('flottante', flott, 'iter_max', 80, 'margine', 0.01);
        [x, info] = ik_risolvi(mod, x, comp, opz);
        K = cinematica_diretta(mod, x.Rb, x.pb, x.qj);
        D = vista_g1_aggiorna(V, K);
        mostra(K, info, D);
        ultimo = x;
    end
    function mostra(K, info, D)
        q = x.qj;
        dati = [mod.giunti(:), num2cell(round(rad2deg(q), 1)), num2cell(round(rad2deg(mod.qmin), 0)), ...
                num2cell(round(rad2deg(mod.qmax), 0))];
        tg.Data = dati;
        removeStyle(tg);
        al_limite = find(q <= mod.qmin + 0.011 | q >= mod.qmax - 0.011);
        if ~isempty(al_limite)
            addStyle(tg, uistyle('BackgroundColor', [1 0.8 0.8]), 'row', al_limite);
        end
        righe = {};
        for k = 1:numel(chiavi)
            [R, p] = posa_frame(mod, K, frame{k});
            e = '';
            if T(k).attivo || (strcmp(dd_modo.Value, 'Feet on ground') && endsWith(chiavi{k}, 'foot'))
                e = sprintf('  (error %.1f mm, %.1f°)', 1e3 * norm(p - T(k).p), rad2deg(norm(so3_log(T(k).R * R'))));
            end
            righe{end + 1} = sprintf('%-11s [%6.3f %6.3f %6.3f] m%s', etichette_tcp{k}, p, e); %#ok<AGROW>
        end
        if info.convergenza, st = 'converged'; else, st = 'best compromise (target not fully reachable)'; end
        righe{end + 1} = sprintf('IK: %s, %d iterations', st, info.iterazioni);
        if ~isempty(D)
            if D.min < 0, righe{end + 1} = sprintf('COLLISION: min capsule distance %.0f mm', 1e3 * D.min);
            else, righe{end + 1} = sprintf('No collision (min capsule distance %.0f mm)', 1e3 * D.min); end
        end
        stato.Value = righe;
        assignin('base', 'g1_ik', struct('x', x, 'q', q, 'joints', {mod.giunti}));
    end
    function salva_posa()
        [f, d] = uiputfile({'*.json', 'JSON pose (*.json)'; '*.csv', 'CSV (*.csv)'}, 'Save pose', ...
                           fullfile(P.exports, 'g1_pose.json'));
        if isequal(f, 0), return; end
        try
            g1_save_pose(fullfile(d, f), mod, x.qj);
            stato.Value = [stato.Value; {['Saved ' fullfile(d, f)]}];
        catch ME
            uialert(fig, ME.message, 'Could not save the pose');
        end
    end
    function carica_posa()
        [f, d] = uigetfile({'*.json;*.csv', 'Pose files (*.json, *.csv)'}, 'Load pose', P.exports);
        if isequal(f, 0), return; end
        try
            [qn, info] = g1_load_pose(fullfile(d, f), mod);
            riparti(qn);
            stato.Value = [stato.Value; {sprintf('Loaded %s (%d joints not in the file, %d clipped to their limits)', ...
                                                 f, numel(info.missing), numel(info.clipped))}];
        catch ME
            uialert(fig, ME.message, 'Could not load the pose');
        end
    end
    function esporta()
        file = fullfile(P.exports, 'pose_ik.csv');
        scrivi_pose_csv(file, mod, [0 1], [ultimo.pb ultimo.pb], cat(3, ultimo.Rb, ultimo.Rb), [ultimo.qj ultimo.qj]);
        stato.Value = [stato.Value; {sprintf('Saved %s', file); 'View it with: python3 python/view_pose.py --csv exports/pose_ik.csv'}];
    end
end

function s = onoff(v)
if v, s = 'on'; else, s = 'off'; end
end
