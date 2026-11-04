function R = so3_exp(w)
%SO3_EXP  Esponenziale su SO(3): rotazione associata al vettore w (asse*angolo).
th = norm(w);
S = skew(w);
if th < 1e-10
    R = eye(3) + S + 0.5 * (S * S);
else
    R = eye(3) + (sin(th) / th) * S + ((1 - cos(th)) / th^2) * (S * S);
end
end
