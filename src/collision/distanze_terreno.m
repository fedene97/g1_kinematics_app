function D = distanze_terreno(C, K, z0, escludi)
%DISTANZE_TERRENO  Distanza delle capsule dal piano orizzontale z = z0.
%   D = distanze_terreno(C, K, z0, escludi)
%   escludi: nomi dei corpi da non controllare (default i piedi, che poggiano).
%   D.d (1 x m): quota del punto più basso della superficie della capsula meno z0
%   D.punto (3 x m): estremo più basso dell'asse; D.capsula: indici; D.min.
if nargin < 3 || isempty(z0), z0 = 0; end
if nargin < 4, escludi = {'left_ankle_roll_link', 'right_ankle_roll_link'}; end
idx = find(~ismember({C.capsula.corpo_nome}, escludi));
m = numel(idx);
D.capsula = idx; D.d = zeros(1, m); D.punto = zeros(3, m);
for i = 1:m
    c = C.capsula(idx(i));
    R = K.R{c.corpo + 1}; p = K.p{c.corpo + 1};
    a = p + R * c.a; b = p + R * c.b;
    if a(3) <= b(3), D.punto(:, i) = a; else, D.punto(:, i) = b; end
    D.d(i) = D.punto(3, i) - c.raggio - z0;
end
if m > 0, D.min = min(D.d); else, D.min = inf; end
end
