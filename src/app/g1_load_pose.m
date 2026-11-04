function [q, info] = g1_load_pose(file, mod)
%G1_LOAD_POSE  Read a joint configuration written by g1_save_pose.
%   [q, info] = g1_load_pose(file, mod) returns q (n x 1, rad, in the model
%   order, clipped to the joint limits). Joints missing from the file stay at 0.
%   info.missing: names of the joints not found; info.clipped: names of the joints
%   that were outside their limits.
[~, ~, ext] = fileparts(file);
q = zeros(mod.n, 1);
found = false(mod.n, 1);
switch lower(ext)
    case '.json'
        s = jsondecode(fileread(file));
        if isfield(s, 'joints'), J = s.joints; else, J = s; end
        fac = pi / 180;
        if isfield(s, 'unit') && strcmpi(char(s.unit), 'rad'), fac = 1; end
        for k = 1:mod.n
            if isfield(J, mod.giunti{k})
                q(k) = double(J.(mod.giunti{k})) * fac;
                found(k) = true;
            end
        end
    case '.csv'
        T = readtable(file, 'TextType', 'string');
        names = string(T.joint);
        if ismember('deg', T.Properties.VariableNames)
            v = T.deg * pi / 180;
        elseif ismember('rad', T.Properties.VariableNames)
            v = T.rad;
        else
            error('g1_load_pose:csv', 'The CSV needs a "joint" column and a "deg" or "rad" column.');
        end
        for k = 1:mod.n
            i = find(names == string(mod.giunti{k}), 1);
            if ~isempty(i), q(k) = v(i); found(k) = true; end
        end
    otherwise
        error('g1_load_pose:ext', 'Use a .json or .csv file (got "%s").', ext);
end
if ~any(found)
    error('g1_load_pose:empty', 'No G1 joint names found in %s.', file);
end
fuori = q < mod.qmin | q > mod.qmax;
q = min(max(q, mod.qmin), mod.qmax);
info.missing = mod.giunti(~found);
info.clipped = mod.giunti(fuori & found);
end
