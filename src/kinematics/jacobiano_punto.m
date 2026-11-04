function J = jacobiano_punto(mod, K, k, rP)
%JACOBIANO_PUNTO  Jacobiano geometrico (6 x 6+n) di un punto del corpo k.
%   [dp_P; w_k] = J * nu, con nu = [v_b; w_b; dq] (velocità della base nel suo
%   frame) e assi del frame mondo (LOCAL_WORLD_ALIGNED di Pinocchio).
%   rP: punto nel frame del corpo k (default origine).
%     colonne della base:  [Rb, -S(pP - pb) Rb; 0, Rb]
%     colonna del giunto j antenato di k:  [z_j x (pP - p_j); z_j],  z_j = R_j a_j
n = mod.n;
if nargin < 4 || isempty(rP), rP = zeros(3, 1); end
if K.simbolico
    rP = costante_sym(rP);
    J = sym(zeros(6, 6 + n));
else
    J = zeros(6, 6 + n);
end
Rb = K.R{1}; pb = K.p{1};
pP = K.p{k + 1} + K.R{k + 1} * rP(:);
J(1:3, 1:3) = Rb;
J(1:3, 4:6) = -skew(pP - pb) * Rb;
J(4:6, 4:6) = Rb;
if k > 0
    for j = mod.antenati{k}
        z = K.R{j + 1} * mod.asse(:, j);
        J(1:3, 6 + j) = cross(z, pP - K.p{j + 1});
        J(4:6, 6 + j) = z;
    end
end
end
