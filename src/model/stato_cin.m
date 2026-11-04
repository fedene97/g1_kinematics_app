function x = stato_cin(mod, varargin)
%STATO_CIN  Configurazione del robot: x.Rb (3x3), x.pb (3x1), x.qj (nx1).
%   x = stato_cin(mod, 'pb', [0;0;0.79], 'qj', q, 'Rb', R)
x.Rb = eye(3);
x.pb = zeros(3, 1);
x.qj = zeros(mod.n, 1);
for i = 1:2:numel(varargin)
    x.(varargin{i}) = varargin{i + 1};
end
x.pb = x.pb(:);
x.qj = x.qj(:);
end
