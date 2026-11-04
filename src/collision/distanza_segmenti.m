function [d, c1, c2, s, t] = distanza_segmenti(p1, q1, p2, q2)
%DISTANZA_SEGMENTI  Distanza minima tra i segmenti p1-q1 e p2-q2 e punti più vicini.
%   c1 = p1 + s (q1 - p1), c2 = p2 + t (q2 - p2), d = |c1 - c2| (Ericson, 2005).
d1 = q1 - p1; d2 = q2 - p2; r = p1 - p2;
a = d1' * d1; e = d2' * d2; f = d2' * r; tol = 1e-12;
if a <= tol && e <= tol
    s = 0; t = 0;
elseif a <= tol
    s = 0; t = min(max(f / e, 0), 1);
else
    c = d1' * r;
    if e <= tol
        t = 0; s = min(max(-c / a, 0), 1);
    else
        b = d1' * d2; den = a * e - b * b;
        if den > tol, s = min(max((b * f - c * e) / den, 0), 1); else, s = 0; end
        t = (b * s + f) / e;
        if t < 0
            t = 0; s = min(max(-c / a, 0), 1);
        elseif t > 1
            t = 1; s = min(max((b - c) / a, 0), 1);
        end
    end
end
c1 = p1 + d1 * s;
c2 = p2 + d2 * t;
d = norm(c1 - c2);
end
