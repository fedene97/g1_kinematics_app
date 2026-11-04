function R = rot_asse(a, q)
%ROT_ASSE  Rotazione di un angolo q attorno all'asse unitario a.
%   Per assi allineati con x, y o z (tutti i giunti del G1) restituisce la
%   rotazione elementare esatta, così le espressioni simboliche restano pulite;
%   altrimenti usa la formula di Rodrigues. q può essere numerico o sym.
a = a(:);
[m, i] = max(abs(a));
if abs(m - 1) < 1e-12 && (sum(abs(a)) - m) < 1e-12
    s  = sign(a(i));
    c  = cos(q);
    sn = s * sin(q);
    switch i
        case 1, R = [1 0 0; 0 c -sn; 0 sn c];
        case 2, R = [c 0 sn; 0 1 0; -sn 0 c];
        case 3, R = [c -sn 0; sn c 0; 0 0 1];
    end
else
    S = skew(a / norm(a));
    R = eye(3) + sin(q) * S + (1 - cos(q)) * (S * S);
end
end
