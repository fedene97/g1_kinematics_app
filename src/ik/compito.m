function c = compito(tipo, varargin)
%COMPITO  Define a task for the inverse kinematics (ik_risolvi).
%   c = compito('frame', 'nome', 'left_sole', 'R', Rd, 'p', pd, ...)
%       pose of a frame (URDF frame name); an empty R or p = component not imposed
%   c = compito('com', 'p', pd, 'maschera', [1 1 0])
%       centre of mass (here only x and y)
%   c = compito('postura', 'q', qref, 'peso', 0.05)
%       regularisation towards a posture (does not enter the stopping criterion)
%   c = compito('collisioni', 'modello', C, 'distanza', 0.03, 'terreno', true)
%       keeps the capsules of the collision model C (see g1_model) apart: every
%       pair closer than 'distanza' [m] is pushed back to 'distanza'. With
%       'terreno' the distance from the plane z = 'z_terreno' is also kept (bodies
%       listed in 'escludi_terreno' are excluded; default: the feet, which rest
%       on the ground)
%   Common options:
%       'priorita'    1 = highest (default 1); higher levels act in the null space
%                     of the previous ones
%       'peso'        scalar or vector (one entry per active row)
%       'maschera'    active components: 6 for 'frame' ([linear; angular]), 3 for 'com'
%       'giunti'      for 'postura': indices of the joints involved (default all)
%       'convergenza' false: the task does not enter the stopping criterion
c = struct('tipo', tipo, 'nome', '', 'R', [], 'p', [], 'q', [], 'giunti', [], ...
           'maschera', [], 'peso', 1, 'priorita', 1, 'convergenza', true, ...
           'modello', [], 'distanza', 0.03, 'terreno', false, 'z_terreno', 0, ...
           'escludi_terreno', {{'left_ankle_roll_link', 'right_ankle_roll_link'}});
dato = false;
for i = 1:2:numel(varargin)
    if ~isfield(c, varargin{i})
        error('compito:opzione', 'Opzione "%s" non prevista.', varargin{i});
    end
    c.(varargin{i}) = varargin{i + 1};
    if strcmp(varargin{i}, 'convergenza'), dato = true; end
end
if strcmp(tipo, 'postura') && ~dato
    c.convergenza = false;
end
if ~any(strcmp(tipo, {'frame', 'com', 'postura', 'collisioni'}))
    error('compito:tipo', 'Tipo di compito "%s" non previsto.', tipo);
end
end
