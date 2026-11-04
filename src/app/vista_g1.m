function V = vista_g1(ax, mod, opz)
%VISTA_G1  Oggetti grafici del G1 in un asse, da aggiornare con vista_g1_aggiorna.
%   V = vista_g1(ax, mod, opz)
%   Crea una volta sola: mesh CAD (STL dell'URDF, semplificate), capsule di
%   collisione, scheletro, terne dei TCP, baricentro e pavimento. Mesh e capsule
%   sono figli di una hgtransform per corpo: aggiornare la posa costa solo 30
%   matrici. Opzioni: mesh (true), capsule (modello C o []), riduzione (0.15:
%   frazione di facce mantenute), colore_mesh.
%   Gruppi per la visibilità: 'gambe', 'tronco' (pelvis, vita, torso, testa), 'braccia'.
d = struct('mesh', true, 'capsule', [], 'riduzione', 0.15, 'colore_mesh', [0.72 0.74 0.78], ...
           'cartella_urdf', '', 'barra', []);
if nargin < 3, opz = struct(); end
opz = completa_opzioni(opz, d);
n = mod.n;
cla(ax); hold(ax, 'on');
V.ax = ax; V.mod = mod;
V.hg = gobjects(1, n + 1);
for k = 0:n, V.hg(k + 1) = hgtransform('Parent', ax); end
V.gruppo = repmat({'tronco'}, 1, n + 1);
for k = 1:n
    nm = mod.giunti{k};
    if contains(nm, {'hip', 'knee', 'ankle'}), V.gruppo{k + 1} = 'gambe';
    elseif contains(nm, {'shoulder', 'elbow', 'wrist'}), V.gruppo{k + 1} = 'braccia'; end
end
% ---- mesh CAD
V.mesh = gobjects(0); V.mesh_corpo = [];
if opz.mesh
    if isempty(opz.cartella_urdf), opz.cartella_urdf = fileparts(mod.file_urdf); end
    U = leggi_urdf(mod.file_urdf);
    tot = numel(U.link);
    for i = 1:tot
        L = U.link(i);
        if ~isempty(opz.barra), opz.barra.Value = i / tot; end
        f = find(strcmp(mod.nomi_frame, L.nome), 1);
        if isempty(f) || isempty(L.mesh), continue; end
        fr = mod.frame(f);
        for m = 1:numel(L.mesh)
            [~, b, e] = fileparts(L.mesh(m).file);
            file = fullfile(opz.cartella_urdf, 'meshes', [b e]);
            if ~isfile(file), continue; end
            T = stlread(file);
            F = T.ConnectivityList; Pm = T.Points;
            if opz.riduzione < 1 && size(F, 1) > 500
                [F, Pm] = reducepatch(F, Pm, opz.riduzione);
            end
            Pm = (fr.R * (rpy2rot(L.mesh(m).rpy) * (Pm' .* L.mesh(m).scala(:)) + L.mesh(m).xyz) + fr.p)';
            V.mesh(end + 1) = patch('Faces', F, 'Vertices', Pm, 'Parent', V.hg(fr.corpo + 1), ...
                'FaceColor', opz.colore_mesh, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
                'AmbientStrength', 0.4, 'DiffuseStrength', 0.7, 'SpecularStrength', 0.2);
            V.mesh_corpo(end + 1) = fr.corpo;
        end
    end
    try
        light(ax, 'Position', [1 -1 2], 'Style', 'infinite');
        light(ax, 'Position', [-1 1 1], 'Style', 'infinite');
    catch
    end
end
% ---- capsule (nel frame del corpo)
V.cap = gobjects(0); V.cap_indice = []; V.cap_corpo = [];
V.C = opz.capsule;
if ~isempty(opz.capsule)
    [xs, ys, zs] = sphere(12);
    for i = 1:numel(opz.capsule.capsula)
        c = opz.capsule.capsula(i);
        par = V.hg(c.corpo + 1);
        v = c.b - c.a; Lc = norm(v);
        if Lc > 1e-9
            [xc, yc, zc] = cylinder(c.raggio, 16); zc = zc * Lc;
            w = v / Lc; t = null(w'); Rc = [t(:, 1), t(:, 2), w];
            Pc = Rc * [xc(:)'; yc(:)'; zc(:)'] + c.a;
            V.cap(end + 1) = surf(reshape(Pc(1, :), size(xc)), reshape(Pc(2, :), size(xc)), ...
                reshape(Pc(3, :), size(xc)), 'Parent', par, 'EdgeColor', 'none'); %#ok<AGROW>
            V.cap_indice(end + 1) = i; V.cap_corpo(end + 1) = c.corpo;
        end
        for e = {c.a, c.b}
            V.cap(end + 1) = surf(c.raggio * xs + e{1}(1), c.raggio * ys + e{1}(2), ...
                c.raggio * zs + e{1}(3), 'Parent', par, 'EdgeColor', 'none'); %#ok<AGROW>
            V.cap_indice(end + 1) = i; V.cap_corpo(end + 1) = c.corpo;
        end
    end
    set(V.cap, 'FaceColor', [0.5 0.6 0.75], 'FaceAlpha', 0.25);
end
% ---- scheletro, TCP, baricentro, pavimento
V.scheletro = plot3(ax, NaN, NaN, NaN, '-', 'Color', [0 0.447 0.741], 'LineWidth', 2);
V.nodi = plot3(ax, NaN, NaN, NaN, 'o', 'Color', [0 0.447 0.741], 'MarkerSize', 4, ...
               'MarkerFaceColor', [0 0.447 0.741]);
V.tcp_nomi = {'pelvis', 'torso', 'head', 'left_hand', 'right_hand', 'left_foot', 'right_foot'};
V.tcp_frame = {mod.nome_base, 'torso_link', 'head_link', 'left_rubber_hand', 'right_rubber_hand', ...
               'left_sole', 'right_sole'};
for i = 1:numel(V.tcp_frame)
    if ~any(strcmp(mod.nomi_frame, V.tcp_frame{i}))
        V.tcp_frame{i} = strrep(strrep(V.tcp_frame{i}, 'rubber_hand', 'wrist_yaw_link'), 'head_link', 'torso_link');
    end
end
colori = [0.8 0 0; 0 0.6 0; 0 0 0.8];
V.terne = gobjects(numel(V.tcp_nomi), 3); V.etichette = gobjects(1, numel(V.tcp_nomi));
for i = 1:numel(V.tcp_nomi)
    for a = 1:3, V.terne(i, a) = plot3(ax, NaN, NaN, NaN, '-', 'Color', colori(a, :), 'LineWidth', 1.8); end
    V.etichette(i) = text(ax, 0, 0, 0, strrep(V.tcp_nomi{i}, '_', ' '), 'FontSize', 8);
end
V.cdm = plot3(ax, NaN, NaN, NaN, 'kp', 'MarkerSize', 11, 'MarkerFaceColor', 'y');
V.cdm_terra = plot3(ax, NaN, NaN, NaN, 'k:');
[X, Y] = meshgrid([-0.6 0.6], [-0.6 0.6]);
V.terra = surf(ax, X, Y, zeros(2), 'FaceColor', [0.88 0.88 0.88], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
axis(ax, 'equal'); grid(ax, 'on'); view(ax, 40, 15);
xlim(ax, [-0.6 0.6]); ylim(ax, [-0.6 0.6]); zlim(ax, [0 1.45]);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
V.visibile = struct('scheletro', true, 'mesh', opz.mesh, 'capsule', false, 'tcp', true, ...
                    'gambe', true, 'tronco', true, 'braccia', true, 'alfa_mesh', 1);
V = vista_g1_visibilita(V, V.visibile);
end
