function h = disegna_posa(mod, K, ax, colore)
%DISEGNA_POSA  Schema a segmenti del robot (origini dei corpi, mani, piante).
if nargin < 3 || isempty(ax), ax = gca; end
if nargin < 4, colore = [0 0.4470 0.7410]; end
hold(ax, 'on');
h = gobjects(0);
for k = 1:mod.n
    a = K.p{mod.padre(k) + 1};
    b = K.p{k + 1};
    h(end + 1) = plot3(ax, [a(1) b(1)], [a(2) b(2)], [a(3) b(3)], '-', ...
                       'Color', colore, 'LineWidth', 1.5); %#ok<AGROW>
end
P0 = cell2mat(K.p);
h(end + 1) = plot3(ax, P0(1, :), P0(2, :), P0(3, :), 'o', 'Color', colore, ...
                   'MarkerSize', 3, 'MarkerFaceColor', colore);
% frame fissi terminali (mani, testa, sensori): segmento dal corpo
for i = 1:numel(mod.frame)
    fr = mod.frame(i);
    if ~strcmp(fr.tipo, 'fisso') || norm(fr.p) < 0.03, continue; end
    a = K.p{fr.corpo + 1};
    b = a + K.R{fr.corpo + 1} * fr.p;
    h(end + 1) = plot3(ax, [a(1) b(1)], [a(2) b(2)], [a(3) b(3)], ':', 'Color', colore); %#ok<AGROW>
end
% piante
for i = 1:numel(mod.piedi)
    pd = mod.piedi(i);
    c = pd.centro;
    [~, o] = sort(atan2(pd.vertici(2, :) - c(2), pd.vertici(1, :) - c(1)));
    V = K.p{pd.corpo + 1} + K.R{pd.corpo + 1} * pd.vertici(:, [o, o(1)]);
    h(end + 1) = plot3(ax, V(1, :), V(2, :), V(3, :), '-', 'Color', colore, 'LineWidth', 1); %#ok<AGROW>
end
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
end
