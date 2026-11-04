function D = vista_g1_aggiorna(V, K)
%VISTA_G1_AGGIORNA  Aggiorna la vista del G1 (vista_g1) per la configurazione K.
%   Restituisce le distanze tra le capsule (D, se il modello è presente) e colora
%   in rosso le capsule in collisione.
mod = V.mod; n = mod.n;
for k = 0:n
    [U, ~, W] = svd(K.R{k + 1});                % rotazione esattamente ortonormale
    V.hg(k + 1).Matrix = [U * W', K.p{k + 1}; 0 0 0 1];
end
% scheletro: segmenti padre-figlio
X = nan(3, 3 * n);
for k = 1:n
    X(:, 3 * k - 2) = K.p{mod.padre(k) + 1};
    X(:, 3 * k - 1) = K.p{k + 1};
end
set(V.scheletro, 'XData', X(1, :), 'YData', X(2, :), 'ZData', X(3, :));
P0 = cell2mat(K.p);
set(V.nodi, 'XData', P0(1, :), 'YData', P0(2, :), 'ZData', P0(3, :));
for i = 1:numel(V.tcp_frame)
    [R, p] = posa_frame(mod, K, V.tcp_frame{i});
    for a = 1:3
        e = p + R(:, a) * 0.07;
        set(V.terne(i, a), 'XData', [p(1) e(1)], 'YData', [p(2) e(2)], 'ZData', [p(3) e(3)]);
    end
    V.etichette(i).Position = (p + [0; 0; 0.035])';
end
pc = baricentro(mod, K);
set(V.cdm, 'XData', pc(1), 'YData', pc(2), 'ZData', pc(3));
set(V.cdm_terra, 'XData', [pc(1) pc(1)], 'YData', [pc(2) pc(2)], 'ZData', [0 pc(3)]);
D = [];
if ~isempty(V.C)
    D = distanze_capsule(V.C, K);
    rosse = unique(D.coppie(:, D.d < 0));
    if ~isempty(V.cap)
        in_col = ismember(V.cap_indice, rosse);
        set(V.cap(~in_col), 'FaceColor', [0.5 0.6 0.75]);
        set(V.cap(in_col), 'FaceColor', [0.9 0.1 0.1]);
    end
end
end
