function [mod, C, P] = g1_model()
%G1_MODEL  Kinematic model and collision capsules of the G1, built from the URDF.
%   [mod, C, P] = g1_model() reads urdf/g1_description/g1_29dof.urdf, builds the
%   kinematic model (mod) and the collision capsules fitted to the meshes (C) and
%   returns them together with the paths (P, see g1_setup). Everything is built
%   once per MATLAB session (about a second) and nothing is written to disk.
%   If the meshes are missing the capsules are skipped (C = []).
persistent cache
if isempty(cache)
    P = g1_setup();
    if ~isfile(P.urdf)
        error('g1_model:urdf', ['URDF not found: %s\n' ...
            'The folder urdf/g1_description (g1_29dof.urdf + meshes/) must be inside the repository folder.'], P.urdf);
    end
    t0 = tic;
    mod = crea_modello_cinematico(P.urdf);
    C = [];
    try
        C = crea_capsule(mod, P.urdf_dir, struct('stampa', false));
    catch ME
        warning('g1_model:capsules', 'Collision capsules not built (%s). The apps still work without collision checking.', ME.message);
    end
    fprintf('G1 model ready: %d joints, %d capsules (%.1f s)\n', mod.n, numel_capsules(C), toc(t0));
    cache = struct('mod', mod, 'C', C, 'P', P);
end
mod = cache.mod;
C = cache.C;
P = cache.P;
end

function n = numel_capsules(C)
if isempty(C), n = 0; else, n = numel(C.capsula); end
end
