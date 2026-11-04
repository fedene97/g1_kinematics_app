function [x, info] = ik_risolvi(mod, x0, compiti, opz)
%IK_RISOLVI  Inverse kinematics with priorities, selective damping and joint limits.
%   [x, info] = ik_risolvi(mod, x0, compiti, opz)
%   x0: initial configuration (stato_cin); compiti: array of tasks (compito).
%   Returns x (x.pb, x.Rb, x.qj) and info (info.convergenza, info.iterazioni,
%   info.tentativi, info.errori = [position error; orientation error] of each
%   task, info.tempo, info.storico).
%   Options (opz, all optional):
%       flottante   free base (default true); false = fixed pelvis
%       giunti      free joints (default all)
%       iter_max    iterations per attempt (300)
%       lambda      minimum damping (1e-6)
%       eps_sing    singular-value threshold below which the damping grows (0.05)
%       lambda_max  maximum damping, at zero singular value (0.05)
%       passo_max   maximum increment per component (0.3)
%       tol_pos     [m] (1e-6), tol_rot [rad] (1e-6)
%       margine     distance kept from the joint limits [rad] (0)
%       riavvii     extra attempts from random configurations of the free joints
%                   if the first one does not converge (0)
%   Step at every iteration, with variables d = [dv; dw; dq] and levels l = 1, 2, ...:
%       d_l = d_{l-1} + (J_l N_{l-1})^# (e_l - J_l d_{l-1}),
%       N_l = N_{l-1} - V_r V_r'   (V_r: directions with singular value > eps_sing)
%   with ^# the selectively damped pseudo-inverse: for every singular value s,
%       s / (s^2 + lambda^2 + lambda_max^2 * max(0, 1 - (s/eps_sing)^2)).
%   Joints that the step would take beyond their limits are stopped at the limit
%   and removed from the variables (active set). A step is accepted only if it
%   reduces the priority-weighted error (halving line search); if no step reduces
%   it the attempt stops (local minimum or incompatible tasks).
d = struct('flottante', true, 'giunti', 1:mod.n, 'iter_max', 300, 'lambda', 1e-6, ...
           'eps_sing', 0.05, 'lambda_max', 0.05, 'passo_max', 0.3, ...
           'tol_pos', 1e-6, 'tol_rot', 1e-6, 'margine', 0, 'riavvii', 0);
if nargin < 4, opz = struct(); end
opz = completa_opzioni(opz, d);

nv = 6 + mod.n;
attive = false(nv, 1);
if opz.flottante, attive(1:6) = true; end
attive(6 + opz.giunti) = true;
qmin = mod.qmin + opz.margine;
qmax = mod.qmax - opz.margine;
prio = [compiti.priorita];
livelli = unique(prio);
% peso di ogni livello nell'errore usato per accettare i passi
peso_liv = 10.^(3 * (numel(livelli) - (1:numel(livelli))));

t0 = tic;
info.storico = zeros(2 * numel(compiti), 0);
info.iterazioni = 0;
info.tentativi = 0;
migliore = [];
for tent = 0:opz.riavvii
    xt = x0;
    if tent > 0
        lib = opz.giunti;
        lo = qmin(lib); hi = qmax(lib);
        lo(~isfinite(lo)) = -pi; hi(~isfinite(hi)) = pi;
        xt.qj(lib) = lo + (hi - lo) .* rand(numel(lib), 1);
    end
    [xt, it_t, conv_t, err_t, f_t, st_t] = un_tentativo(mod, xt, compiti, opz, attive, ...
        qmin, qmax, prio, livelli, peso_liv);
    info.iterazioni = info.iterazioni + it_t;
    info.tentativi = tent + 1;
    if tent == 0, info.storico = st_t; end
    if isempty(migliore) || conv_t || f_t < migliore.f
        migliore = struct('x', xt, 'conv', conv_t, 'err', err_t, 'f', f_t);
    end
    if conv_t, break; end
end
x = migliore.x;
info.convergenza = migliore.conv;
info.errori = migliore.err;
info.tempo = toc(t0);
end

% ======================================================================
function [x, it, convok, err, f, storico] = un_tentativo(mod, x, compiti, opz, attive, ...
    qmin, qmax, prio, livelli, peso_liv)
storico = zeros(2 * numel(compiti), opz.iter_max + 1);
[J, e, conv, err, f] = valuta(mod, x, compiti, opz, prio, livelli, peso_liv);
convok = false;
for it = 0:opz.iter_max
    storico(:, it + 1) = err;
    if all(conv), convok = true; break; end
    if it == opz.iter_max, break; end
    D = passo_gerarchico(J, e, prio, livelli, attive, x.qj, qmin, qmax, opz);
    m = max(abs(D));
    if m > opz.passo_max, D = D * (opz.passo_max / m); end
    a = 1;
    accettato = false;
    for bt = 1:10
        xn = applica_incremento(x, a * D);
        lib = opz.giunti;
        xn.qj(lib) = min(max(xn.qj(lib), qmin(lib)), qmax(lib));
        [Jn, en, convn, errn, fn] = valuta(mod, xn, compiti, opz, prio, livelli, peso_liv);
        if fn < f
            accettato = true;
            break
        end
        a = a / 2;
    end
    if ~accettato, break; end          % nessun passo riduce l'errore
    x = xn; J = Jn; e = en; conv = convn; err = errn; f = fn;
end
storico = storico(:, 1:it + 1);
end

function [Jc, ec, conv, err, f] = valuta(mod, x, compiti, opz, prio, livelli, peso_liv)
K = cinematica_diretta(mod, x.Rb, x.pb, x.qj);
nt = numel(compiti);
n = mod.n;
Jc = cell(1, nt); ec = cell(1, nt);
conv = true(1, nt);
err = zeros(2 * nt, 1);
f = 0;
for i = 1:nt
    c = compiti(i);
    switch c.tipo
        case 'frame'
            fi = trova_frame(mod, c.nome);
            [R, p] = posa_frame(mod, K, fi);
            J = jacobiano_frame(mod, K, fi, 'mondo');
            e = zeros(6, 1);
            m = false(6, 1);
            if ~isempty(c.p), e(1:3) = c.p(:) - p; m(1:3) = true; end
            if ~isempty(c.R), e(4:6) = so3_log(c.R * R'); m(4:6) = true; end
            if ~isempty(c.maschera), m = m & logical(c.maschera(:)); end
            ep = norm(e(1:3) .* m(1:3));
            er = norm(e(4:6) .* m(4:6));
        case 'com'
            [pc, J] = baricentro(mod, K);
            e = c.p(:) - pc;
            m = true(3, 1);
            if ~isempty(c.maschera), m = m & logical(c.maschera(:)); end
            ep = norm(e .* m);
            er = 0;
        case 'collisioni'
            Dc = distanze_capsule(c.modello, K);
            att = find(Dc.d < c.distanza);
            J = zeros(numel(att), 6 + n);
            e = zeros(numel(att), 1);
            for r = 1:numel(att)
                a = att(r);
                k1 = c.modello.capsula(Dc.coppie(1, a)).corpo;
                k2 = c.modello.capsula(Dc.coppie(2, a)).corpo;
                J1 = jacobiano_punto(mod, K, k1, K.R{k1 + 1}' * (Dc.c1(:, a) - K.p{k1 + 1}));
                J2 = jacobiano_punto(mod, K, k2, K.R{k2 + 1}' * (Dc.c2(:, a) - K.p{k2 + 1}));
                J(r, :) = Dc.n(:, a)' * (J1(1:3, :) - J2(1:3, :));
                e(r) = c.distanza - Dc.d(a);
            end
            dmin = Dc.min;
            if c.terreno
                Dt = distanze_terreno(c.modello, K, c.z_terreno, c.escludi_terreno);
                at = find(Dt.d < c.distanza);
                for r = 1:numel(at)
                    a = at(r); kc = c.modello.capsula(Dt.capsula(a)).corpo;
                    Jp = jacobiano_punto(mod, K, kc, K.R{kc + 1}' * (Dt.punto(:, a) - K.p{kc + 1}));
                    J(end + 1, :) = Jp(3, :); %#ok<AGROW>
                    e(end + 1, 1) = c.distanza - Dt.d(a); %#ok<AGROW>
                end
                dmin = min([dmin, Dt.min]);
            end
            m = true(size(J, 1), 1);
            ep = max([0, c.distanza - dmin]);
            er = 0;
            if c.convergenza
                conv(i) = ep <= 1e-5;
            end
        case 'postura'
            J = [zeros(n, 6), eye(n)];
            e = c.q(:) - x.qj;
            m = false(n, 1);
            if isempty(c.giunti), m(:) = true; else, m(c.giunti) = true; end
            ep = norm(e(m));
            er = 0;
    end
    J = J(m, :);
    e = e(m);
    w = c.peso(:);
    J = w .* J;
    e = w .* e;
    Jc{i} = J;
    ec{i} = e;
    err(2 * i - 1) = ep;
    err(2 * i) = er;
    if c.convergenza && ~strcmp(c.tipo, 'collisioni')
        conv(i) = (ep <= opz.tol_pos) && (er <= opz.tol_rot);
    end
    f = f + peso_liv(livelli == prio(i)) * (e' * e);
end
end

function D = passo_gerarchico(J, e, prio, livelli, attive, qj, qmin, qmax, opz)
nv = numel(attive);
blocc = ~attive;
Dfix = zeros(nv, 1);
for rip = 1:10
    D = Dfix;
    N = diag(double(~blocc));
    for l = livelli
        sel = find(prio == l);
        Jl = vertcat(J{sel});
        el = vertcat(e{sel});
        if isempty(Jl), continue; end
        JN = Jl * N;
        [U, S, V] = svd(JN, 'econ');
        s = diag(S);
        l2 = opz.lambda^2 + opz.lambda_max^2 * max(0, 1 - (s / opz.eps_sing).^2);
        D = D + V * ((s ./ (s.^2 + l2)) .* (U' * (el - Jl * D)));
        r = s > opz.eps_sing * 1e-2;               % direzioni usate dal livello
        N = N - V(:, r) * V(:, r)';
    end
    qn = qj + D(7:end);
    viol = ~blocc(7:end) & (qn < qmin | qn > qmax);
    if ~any(viol), break; end
    iv = find(viol);
    blocc(6 + iv) = true;
    Dfix(6 + iv) = min(max(qn(iv), qmin(iv)), qmax(iv)) - qj(iv);
end
end
