function h = disegna_capsule(C, K, ax, evidenzia, alfa)
%DISEGNA_CAPSULE  Disegna le capsule di collisione (superfici semitrasparenti).
%   evidenzia: indici delle capsule da colorare in rosso (es. coppie in collisione).
if nargin < 3 || isempty(ax), ax = gca; end
if nargin < 4, evidenzia = []; end
if nargin < 5, alfa = 0.25; end
hold(ax, 'on');
[xs, ys, zs] = sphere(12);
h = gobjects(0);
for i = 1:numel(C.capsula)
    c = C.capsula(i);
    R = K.R{c.corpo + 1}; p = K.p{c.corpo + 1};
    a = p + R * c.a; b = p + R * c.b; r = c.raggio;
    if any(evidenzia == i), col = [0.85 0.1 0.1]; else, col = [0.5 0.6 0.75]; end
    % cilindro
    v = b - a; Lc = norm(v);
    if Lc > 1e-9
        [xc, yc, zc] = cylinder(r, 16);
        zc = zc * Lc;
        w = v / Lc; t = null(w'); Rc = [t(:, 1), t(:, 2), w];
        P = Rc * [xc(:)'; yc(:)'; zc(:)'] + a;
        h(end + 1) = surf(ax, reshape(P(1, :), size(xc)), reshape(P(2, :), size(xc)), ...
                          reshape(P(3, :), size(xc)), 'FaceColor', col, 'EdgeColor', 'none', ...
                          'FaceAlpha', alfa); %#ok<AGROW>
    end
    for e = {a, b}
        h(end + 1) = surf(ax, r * xs + e{1}(1), r * ys + e{1}(2), r * zs + e{1}(3), ...
                          'FaceColor', col, 'EdgeColor', 'none', 'FaceAlpha', alfa); %#ok<AGROW>
    end
end
axis(ax, 'equal');
end
