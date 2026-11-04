function J = jacobiano_frame(mod, K, f, conv)
%JACOBIANO_FRAME  Jacobiano geometrico (6 x 6+n) di un frame del modello.
%   conv = 'mondo'    (default) velocità dell'origine del frame e velocità
%                     angolare, assi del mondo (LOCAL_WORLD_ALIGNED)
%        = 'locale'   stesse grandezze negli assi del frame (LOCAL)
%        = 'spaziale' twist spaziale: velocità del punto del corpo che passa
%                     per l'origine del mondo (WORLD)
%   Con base fissa le colonne utili sono J(:, 7:end).
if nargin < 4, conv = 'mondo'; end
f  = trova_frame(mod, f);
fr = mod.frame(f);
J  = jacobiano_punto(mod, K, fr.corpo, fr.p);
switch conv
    case 'mondo'
    case 'locale'
        R = posa_frame(mod, K, f);
        J = [R' * J(1:3, :); R' * J(4:6, :)];
    case 'spaziale'
        [~, p] = posa_frame(mod, K, f);
        J(1:3, :) = J(1:3, :) + skew(p) * J(4:6, :);
    otherwise
        error('jacobiano_frame:conv', 'Convenzione "%s" non prevista.', conv);
end
end
