function mod = crea_modello_cinematico(file_urdf)
%CREA_MODELLO_CINEMATICO  Modello cinematico ad albero dall'URDF.
%   mod = crea_modello_cinematico(file) legge l'URDF e costruisce:
%     n, giunti, corpi          giunti attivi (revolute/continuous) e link figli
%     padre (1xn)               corpo padre del corpo k (0 = base)
%     asse (3xn), r (3xn), Rf (3x3xn)
%                               asse del giunto nel frame k, origine del frame k
%                               nel frame del padre, rotazione fissa:
%                               E_k(q) = Rf_k * Rot(asse_k, q), r_k
%     qmin, qmax, vmax, coppia_max (nx1)
%     antenati{k}               giunti da base a k (inclusi), in ordine
%     frame(i)                  tutti i link dell'URDF (attivi e fissi) e i punti
%                               delle piante: nome, corpo, R, p (posa costante
%                               nel frame del corpo attivo), tipo
%     massa (1x(n+1)), cdm (3x(n+1))
%                               massa e baricentro dei corpi dopo il collasso dei
%                               link fissi (colonna 1 = base), per il baricentro
%     piedi(i)                  vertici e centro della pianta (dalle sfere di
%                               collisione di *_ankle_roll_link)
%   Convenzioni identiche al modello dinamico (ws_g1_dynamics): frame del corpo
%   k = frame del giunto k, base = pelvis.

U = leggi_urdf(file_urdf);
nomi_link = {U.link.nome};
radice = setdiff(nomi_link, {U.giunto.figlio}, 'stable');
if numel(radice) ~= 1
    error('crea_modello_cinematico:radice', 'Attesa una sola radice, trovate %d.', numel(radice));
end
radice = radice{1};

% posa di ogni link rispetto al proprio corpo attivo
info = containers.Map('KeyType', 'char', 'ValueType', 'any');
info(radice) = struct('k', 0, 'R', eye(3), 'p', zeros(3, 1));
attivi = {radice};

nG = numel(U.giunto);
fatto = false(1, nG);
n = 0;
giunti = {}; corpi = {}; padre = []; asse = []; r = []; Rf = [];
qmin = []; qmax = []; vmax = []; coppia = [];
cambiato = true;
while cambiato
    cambiato = false;
    for i = 1:nG
        if fatto(i), continue; end
        G = U.giunto(i);
        if ~isKey(info, G.padre), continue; end
        A  = info(G.padre);
        pj = A.p + A.R * G.xyz;           % origine del giunto nel frame del corpo attivo
        Rj = A.R * rpy2rot(G.rpy);
        switch G.tipo
            case {'revolute', 'continuous'}
                n = n + 1;
                giunti{n} = G.nome;  corpi{n} = G.figlio; %#ok<AGROW>
                padre(n) = A.k; %#ok<AGROW>
                r(:, n) = pj; %#ok<AGROW>
                Rf(:, :, n) = Rj; %#ok<AGROW>
                asse(:, n) = G.asse / norm(G.asse); %#ok<AGROW>
                qmin(n, 1) = G.qmin; qmax(n, 1) = G.qmax; %#ok<AGROW>
                vmax(n, 1) = G.vmax; coppia(n, 1) = G.coppia_max; %#ok<AGROW>
                info(G.figlio) = struct('k', n, 'R', eye(3), 'p', zeros(3, 1));
                attivi{end + 1} = G.figlio; %#ok<AGROW>
            case 'fixed'
                info(G.figlio) = struct('k', A.k, 'R', Rj, 'p', pj);
            otherwise
                error('crea_modello_cinematico:tipo', 'Giunto %s di tipo %s non supportato.', G.nome, G.tipo);
        end
        fatto(i) = true;
        cambiato = true;
    end
end
if ~all(fatto)
    warning('crea_modello_cinematico:scollegati', '%d giunti non collegati alla radice.', sum(~fatto));
end

mod.file_urdf = file_urdf;
mod.md5 = md5_file(file_urdf);
mod.nome_robot = U.nome;
mod.nome_base = radice;
mod.n = n;
mod.giunti = giunti;
mod.corpi = corpi;
mod.padre = padre;
mod.asse = asse;
mod.r = r;
mod.Rf = Rf;
mod.qmin = qmin; mod.qmax = qmax; mod.vmax = vmax; mod.coppia_max = coppia;

mod.antenati = cell(1, n);
for k = 1:n
    c = k; L = [];
    while c > 0
        L = [c, L]; %#ok<AGROW>
        c = padre(c);
    end
    mod.antenati{k} = L;
end

% ------------------------------------------- masse (collasso dei link fissi)
massa = zeros(1, n + 1);
h = zeros(3, n + 1);
for i = 1:numel(U.link)
    L = U.link(i);
    if L.massa <= 0 || ~isKey(info, L.nome), continue; end
    A = info(L.nome);
    c = A.p + A.R * L.cdm;
    massa(A.k + 1) = massa(A.k + 1) + L.massa;
    h(:, A.k + 1) = h(:, A.k + 1) + L.massa * c;
end
cdm = zeros(3, n + 1);
nz = massa > 0;
cdm(:, nz) = h(:, nz) ./ massa(nz);
mod.massa = massa;
mod.cdm = cdm;
mod.massa_totale = sum(massa);

% ------------------------------------------------------------------ frame
F = struct('nome', {}, 'corpo', {}, 'R', {}, 'p', {}, 'tipo', {});
for i = 1:numel(nomi_link)
    if ~isKey(info, nomi_link{i}), continue; end
    A = info(nomi_link{i});
    if any(strcmp(attivi, nomi_link{i})), tipo = 'corpo'; else, tipo = 'fisso'; end
    F(end + 1) = struct('nome', nomi_link{i}, 'corpo', A.k, 'R', A.R, 'p', A.p, 'tipo', tipo); %#ok<AGROW>
end

% ------------------------------------------- piante: vertici dalle sfere
mod.piedi = struct('nome', {}, 'link', {}, 'corpo', {}, 'vertici', {}, 'centro', {});
for i = 1:numel(U.link)
    L = U.link(i);
    tok = regexp(L.nome, '^(.*)_ankle_roll_link$', 'tokens', 'once');
    if isempty(tok) || isempty(L.sfere) || ~isKey(info, L.nome), continue; end
    A = info(L.nome);
    V = L.sfere(1:3, :) - [0; 0; 1] * L.sfere(4, :);   % punto più basso di ogni sfera
    V = A.p + A.R * V;
    centro = mean(V, 2);
    mod.piedi(end + 1) = struct('nome', tok{1}, 'link', L.nome, 'corpo', A.k, ...
                                'vertici', V, 'centro', centro);
    F(end + 1) = struct('nome', [tok{1} '_sole'], 'corpo', A.k, 'R', eye(3), ...
                        'p', centro, 'tipo', 'punto'); %#ok<AGROW>
    for v = 1:size(V, 2)
        F(end + 1) = struct('nome', sprintf('%s_sole_%d', tok{1}, v), 'corpo', A.k, ...
                            'R', eye(3), 'p', V(:, v), 'tipo', 'punto'); %#ok<AGROW>
    end
end
mod.frame = F;
mod.nomi_frame = {F.nome};

% controlli
for k = 1:n
    E = Rf(:, :, k);
    if norm(E' * E - eye(3), inf) > 1e-12
        error('crea_modello_cinematico:rotazione', 'Rotazione fissa del giunto %d non ortonormale.', k);
    end
    if padre(k) >= k
        error('crea_modello_cinematico:ordine', 'Ordine dei giunti non topologico (giunto %d).', k);
    end
end
end
