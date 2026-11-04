function C = crea_capsule(mod, cartella_urdf, opz)
%CREA_CAPSULE  Modello di collisione a capsule, adattato alle mesh visive dell'URDF.
%   C = crea_capsule(mod, cartella_urdf, opz)
%   Per ogni corpo attivo (base inclusa) si raccolgono i vertici delle mesh dei suoi
%   link (anche dei link fissi collassati, es. mani finte e testa) nel frame del
%   corpo; la capsula ha asse lungo la direzione principale dei vertici (analisi
%   delle componenti principali), raggio pari al quantile 'quantile' (0.95) delle
%   distanze dei vertici dall'asse ed estremi tali da contenere lo stesso quantile
%   lungo l'asse. Coppie escluse: corpi a 1 o 2 livelli di distanza nella stessa
%   catena e coppie già a contatto in una posa di riposo (opz.pose_riposo).
%   I corpi grandi e larghi (opz.capsule_per_corpo, default torso 3, pelvis 2) sono
%   divisi in più capsule: i vertici vengono separati ricorsivamente con il taglio
%   (a metà lungo uno degli assi principali) che riduce di più il volume totale.
%   C.capsula(i): nome, corpo, corpo_nome, a, b (estremi nel frame del corpo), raggio
%   C.coppie (2 x m): indici delle capsule da controllare
d = struct('quantile', 0.95, 'pose_riposo', [], 'margine_riposo', 0.0, 'stampa', true, ...
           'capsule_per_corpo', struct('torso_link', 3, 'pelvis', 2));
if nargin < 3, opz = struct(); end
opz = completa_opzioni(opz, d);
U = leggi_urdf(mod.file_urdf);
n = mod.n;
V = cell(1, n + 1);
for i = 1:numel(U.link)
    L = U.link(i);
    f = find(strcmp(mod.nomi_frame, L.nome), 1);
    if isempty(f) || isempty(L.mesh), continue; end
    fr = mod.frame(f);
    for m = 1:numel(L.mesh)
        [~, base, est] = fileparts(L.mesh(m).file);
        file = fullfile(cartella_urdf, 'meshes', [base est]);
        if ~isfile(file)
            warning('crea_capsule:mesh', 'Mesh non trovata: %s', file); continue
        end
        T = stlread(file);
        P = T.Points' .* L.mesh(m).scala(:);
        P = rpy2rot(L.mesh(m).rpy) * P + L.mesh(m).xyz;     % frame del link
        P = fr.R * P + fr.p;                                % frame del corpo attivo
        V{fr.corpo + 1} = [V{fr.corpo + 1}, unique(P', 'rows')'];
    end
end
cap = struct('nome', {}, 'corpo', {}, 'corpo_nome', {}, 'a', {}, 'b', {}, 'raggio', {}, 'n_vertici', {});
for k = 0:n
    P = V{k + 1};
    if size(P, 2) < 10, continue; end
    if k == 0, nome = mod.nome_base; else, nome = mod.corpi{k}; end
    nk = 1;
    if isfield(opz.capsule_per_corpo, nome), nk = opz.capsule_per_corpo.(nome); end
    G = dividi_vertici(P, nk, opz.quantile);
    for j = 1:numel(G)
        [a, b, r] = adatta_capsula(G{j}, opz.quantile);
        if numel(G) > 1, nc = sprintf('%s_%d', nome, j); else, nc = nome; end
        cap(end + 1) = struct('nome', nc, 'corpo', k, 'corpo_nome', nome, 'a', a, 'b', b, ...
                              'raggio', r, 'n_vertici', size(G{j}, 2)); %#ok<AGROW>
    end
end
C.capsula = cap;
C.quantile = opz.quantile;
% coppie candidate: tutte tranne i corpi vicini nella stessa catena
nc = numel(cap);
coppie = zeros(2, 0);
for i = 1:nc
    for j = i + 1:nc
        if distanza_albero(mod, cap(i).corpo, cap(j).corpo) <= 2, continue; end
        coppie(:, end + 1) = [i; j]; %#ok<AGROW>
    end
end
% coppie già a contatto nelle pose di riposo: escluse
escluse = zeros(2, 0);
pose = opz.pose_riposo;
if isempty(pose), pose = {zeros(n, 1), postura_casa(mod)}; end
C.coppie = coppie;
tieni = true(1, size(coppie, 2));
for p = 1:numel(pose)
    K = cinematica_diretta(mod, eye(3), zeros(3, 1), pose{p});
    D = distanze_capsule(C, K);
    tieni = tieni & (D.d > opz.margine_riposo);
end
escluse = coppie(:, ~tieni);
C.coppie = coppie(:, tieni);
C.coppie_escluse = escluse;
if opz.stampa
    fprintf('  capsule: %d corpi, %d coppie controllate, %d escluse perché a contatto a riposo\n', ...
            nc, size(C.coppie, 2), size(escluse, 2));
    for e = 1:size(escluse, 2)
        fprintf('    esclusa: %s - %s\n', cap(escluse(1, e)).nome, cap(escluse(2, e)).nome);
    end
end
end

function [a, b, r] = adatta_capsula(P, q)
% capsula sui vertici P (3 x N): asse principale, raggio e estremi al quantile q
c = mean(P, 2);
[W, ~] = eig(cov((P - c)'));
u = W(:, 3);
t = u' * (P - c);
D = P - c - u * t;
r = quantile_semplice(vecnorm(D), q);
t1 = quantile_semplice(t, 1 - q); t2 = quantile_semplice(t, q);
if t2 - t1 > 2 * r
    a = c + u * (t1 + r); b = c + u * (t2 - r);
else
    a = c + u * (t1 + t2) / 2; b = a;                       % sfera
end
end

function G = dividi_vertici(P, k, q)
% divisione ricorsiva in k gruppi: a ogni passo il taglio (a metà lungo un asse
% principale di un gruppo) che riduce di più il volume totale delle capsule
G = {P};
while numel(G) < k
    migliore = [];
    for gi = 1:numel(G)
        Q = G{gi};
        v0 = volume(Q, q);
        c = mean(Q, 2);
        [W, ~] = eig(cov((Q - c)'));
        for ax = 1:3
            t = W(:, ax)' * (Q - c);
            m = median(t);
            A = Q(:, t <= m); B = Q(:, t > m);
            if size(A, 2) < 20 || size(B, 2) < 20, continue; end
            guadagno = v0 - volume(A, q) - volume(B, q);
            if isempty(migliore) || guadagno > migliore.g
                migliore = struct('g', guadagno, 'gi', gi, 'A', A, 'B', B);
            end
        end
    end
    if isempty(migliore), break; end
    G = [G(1:migliore.gi - 1), {migliore.A, migliore.B}, G(migliore.gi + 1:end)];
end
end

function v = volume(P, q)
[a, b, r] = adatta_capsula(P, q);
v = pi * r^2 * (norm(b - a) + 4 / 3 * r);
end

function dd = distanza_albero(mod, i, j)
% numero di giunti tra i corpi i e j (0 = base)
ai = antenati_corpo(mod, i); aj = antenati_corpo(mod, j);
comuni = intersect(ai, aj);
lca = max(comuni);
dd = (numel(ai) - find(ai == lca)) + (numel(aj) - find(aj == lca));
end

function a = antenati_corpo(mod, k)
% corpi dalla base a k inclusi (0 = base)
if k == 0, a = 0; else, a = [0, mod.antenati{k}]; end
end
