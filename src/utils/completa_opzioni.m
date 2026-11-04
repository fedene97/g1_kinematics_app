function o = completa_opzioni(o, d)
%COMPLETA_OPZIONI  Aggiunge a o i campi di d mancanti (valori di default).
if isempty(o), o = struct(); end
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(o, f{i}), o.(f{i}) = d.(f{i}); end
end
end
