function K = cinematica_diretta(mod, Rb, pb, qj)
%CINEMATICA_DIRETTA  Posa di tutti i corpi nel frame mondo (ricorsione in avanti).
%   K = cinematica_diretta(mod, Rb, pb, qj)
%   K.R{k+1}, K.p{k+1}: orientamento e origine del corpo k (k = 0 base) nel mondo
%       R_k = R_p(k) * Rf_k * Rot(a_k, q_k),   p_k = p_p(k) + R_p(k) * r_k
%   Gli ingressi possono essere numerici o simbolici; con ingressi sym le
%   costanti dell'URDF entrano come numeri vpa a 16 cifre (costante_sym).
n = mod.n;
simb = isa(qj, 'sym') || isa(Rb, 'sym') || isa(pb, 'sym');
K.R = cell(1, n + 1);
K.p = cell(1, n + 1);
K.R{1} = Rb;
K.p{1} = pb(:);
for k = 1:n
    j  = mod.padre(k) + 1;
    Rf = mod.Rf(:, :, k);
    r  = mod.r(:, k);
    if simb
        Rf = costante_sym(Rf);
        r  = costante_sym(r);
    end
    if isequal(mod.Rf(:, :, k), eye(3))
        E = rot_asse(mod.asse(:, k), qj(k));
    else
        E = Rf * rot_asse(mod.asse(:, k), qj(k));
    end
    K.R{k + 1} = K.R{j} * E;
    K.p{k + 1} = K.p{j} + K.R{j} * r;
end
K.simbolico = simb;
end
