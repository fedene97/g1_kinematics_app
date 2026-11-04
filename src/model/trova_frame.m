function f = trova_frame(mod, nome)
%TROVA_FRAME  Indice di un frame del modello dal nome (un indice viene restituito com'è).
if isnumeric(nome), f = nome; return; end
f = find(strcmp(mod.nomi_frame, nome), 1);
if isempty(f)
    error('trova_frame:assente', 'Frame "%s" non presente nel modello.', nome);
end
end
