function g1_save_pose(file, mod, q)
%G1_SAVE_POSE  Write a joint configuration to a .json or .csv file.
%   g1_save_pose(file, mod, q): q = joint angles [rad] in the model order.
%   JSON: {"robot": ..., "unit": "deg", "joints": {"left_hip_pitch_joint": 0, ...}}
%   CSV : columns joint, deg, rad
%   The file can be read back with g1_load_pose.
[~, ~, ext] = fileparts(file);
q = q(:);
deg = rad2deg(q);
switch lower(ext)
    case '.json'
        j = struct();
        for k = 1:mod.n
            j.(mod.giunti{k}) = round(deg(k), 4);
        end
        s = struct('robot', 'unitree_g1_29dof', 'unit', 'deg', 'joints', j);
        try
            txt = jsonencode(s, 'PrettyPrint', true);
        catch
            txt = jsonencode(s);
        end
        escrivi(file, txt);
    case '.csv'
        linee = cell(mod.n + 1, 1);
        linee{1} = 'joint,deg,rad';
        for k = 1:mod.n
            linee{k + 1} = sprintf('%s,%.4f,%.6f', mod.giunti{k}, deg(k), q(k));
        end
        escrivi(file, strjoin(linee, newline));
    otherwise
        error('g1_save_pose:ext', 'Use a .json or .csv file name (got "%s").', ext);
end
end

function escrivi(file, txt)
fid = fopen(file, 'w');
if fid < 0, error('g1_save_pose:open', 'Cannot write %s', file); end
c = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', txt);
end
