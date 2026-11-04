function b = bias_punto(mod, K, V, k, rP) %#ok<INUSL>
%BIAS_PUNTO  Termine Jdot*nu di un punto del corpo k: [accelerazione del punto; al_k].
%   È l'accelerazione classica del punto (e l'accelerazione angolare del corpo)
%   con dnu = 0, assi del mondo; V da cinematica_velocita.
if nargin < 5 || isempty(rP), rP = zeros(3, 1); end
s = K.R{k + 1} * rP(:);
a = V.a{k + 1} + cross(V.al{k + 1}, s) + cross(V.w{k + 1}, cross(V.w{k + 1}, s));
b = [a; V.al{k + 1}];
end
