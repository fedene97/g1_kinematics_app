function S = skew(v)
%SKEW  Matrice antisimmetrica: skew(v)*x = cross(v, x). Funziona anche con sym.
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
