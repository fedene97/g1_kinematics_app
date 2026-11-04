function V = vista_g1_visibilita(V, vis)
%VISTA_G1_VISIBILITA  Mostra o nasconde parti della vista: vis con campi scheletro,
%   mesh, capsule, tcp, gambe, tronco, braccia (logici) e alfa_mesh (trasparenza).
V.visibile = completa_opzioni(vis, V.visibile);
v = V.visibile;
onoff = {'off', 'on'};
set([V.scheletro, V.nodi], 'Visible', onoff{v.scheletro + 1});
set([V.terne(:); V.etichette(:); V.cdm; V.cdm_terra], 'Visible', onoff{v.tcp + 1});
gruppi_on = {};
for g = {'gambe', 'tronco', 'braccia'}
    if v.(g{1}), gruppi_on{end + 1} = g{1}; end %#ok<AGROW>
end
if ~isempty(V.mesh)
    on = v.mesh & ismember(V.gruppo(V.mesh_corpo + 1), gruppi_on);
    set(V.mesh(on), 'Visible', 'on', 'FaceAlpha', v.alfa_mesh);
    set(V.mesh(~on), 'Visible', 'off');
end
if ~isempty(V.cap)
    on = v.capsule & ismember(V.gruppo(V.cap_corpo + 1), gruppi_on);
    set(V.cap(on), 'Visible', 'on');
    set(V.cap(~on), 'Visible', 'off');
end
end
