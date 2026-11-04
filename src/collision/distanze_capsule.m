function D = distanze_capsule(C, K, coppie)
%DISTANZE_CAPSULE  Distanze tra le capsule (superfici) per la configurazione K.
%   D.d (1 x m): distanza tra le superfici (negativa = compenetrazione)
%   D.c1, D.c2 (3 x m): punti più vicini sugli assi; D.n (3 x m): versore da 2 a 1
%   D.coppie: indici delle capsule; D.min, D.imin: distanza minima e coppia.
if nargin < 3, coppie = C.coppie; end
m = size(coppie, 2);
D.coppie = coppie;
D.d = zeros(1, m); D.c1 = zeros(3, m); D.c2 = zeros(3, m); D.n = zeros(3, m);
for k = 1:m
    A = C.capsula(coppie(1, k)); B = C.capsula(coppie(2, k));
    Ra = K.R{A.corpo + 1}; pa = K.p{A.corpo + 1};
    Rb = K.R{B.corpo + 1}; pb = K.p{B.corpo + 1};
    [dd, c1, c2] = distanza_segmenti(pa + Ra * A.a, pa + Ra * A.b, pb + Rb * B.a, pb + Rb * B.b);
    D.d(k) = dd - A.raggio - B.raggio;
    D.c1(:, k) = c1; D.c2(:, k) = c2;
    if dd > 1e-12, D.n(:, k) = (c1 - c2) / dd; else, D.n(:, k) = [0; 0; 1]; end
end
if m > 0
    [D.min, D.imin] = min(D.d);
else
    D.min = inf; D.imin = 0;
end
end
