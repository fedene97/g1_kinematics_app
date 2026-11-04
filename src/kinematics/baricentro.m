function [pc, Jc, bc] = baricentro(mod, K, V)
%BARICENTRO  Baricentro del robot, suo Jacobiano (3 x 6+n) e termine Jdot*nu.
%   pc = sum_i m_i (p_i + R_i c_i) / M
%   Jc: colonne della base [Rb, -S(pc - pb) Rb]; colonna del giunto k
%       z_k x (sum_{i nel sottoalbero di k} m_i (pc_i - p_k)) / M
%   bc = Jdot_c*nu (richiede V da cinematica_velocita).
n = mod.n;
simb = K.simbolico;
M = mod.massa_totale;
if simb, M = costante_sym(M); end
Sm = cell(1, n + 1);
Ms = cell(1, n + 1);
pci = cell(1, n + 1);
pc = zeros(3, 1);
for i = 1:n + 1
    ci = mod.cdm(:, i); mi = mod.massa(i);
    if simb, ci = costante_sym(ci); mi = costante_sym(mi); end
    pci{i} = K.p{i} + K.R{i} * ci;
    Sm{i} = mi * pci{i};
    Ms{i} = mi;
    pc = pc + Sm{i};
end
pc = pc / M;
if nargout < 2, return; end

for k = n:-1:1                      % somme sui sottoalberi (figli dopo i padri)
    j = mod.padre(k) + 1;
    Sm{j} = Sm{j} + Sm{k + 1};
    Ms{j} = Ms{j} + Ms{k + 1};
end
if simb, Jc = sym(zeros(3, 6 + n)); else, Jc = zeros(3, 6 + n); end
Rb = K.R{1};
Jc(:, 1:3) = Rb;
Jc(:, 4:6) = -skew(pc - K.p{1}) * Rb;
for k = 1:n
    z = K.R{k + 1} * mod.asse(:, k);
    Jc(:, 6 + k) = cross(z, Sm{k + 1} - Ms{k + 1} * K.p{k + 1}) / M;
end
if nargout < 3, return; end

bc = zeros(3, 1);
for i = 1:n + 1
    b = bias_punto(mod, K, V, i - 1, mod.cdm(:, i));
    bc = bc + mod.massa(i) * b(1:3);
end
bc = bc / mod.massa_totale;
end
