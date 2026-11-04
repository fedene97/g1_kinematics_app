function P = g1_setup()
%G1_SETUP  Add the app folders to the MATLAB path and return the main paths.
%   P = g1_setup() works out the repository location from the position of this
%   file (there are no absolute paths anywhere), adds every folder under src/ to
%   the MATLAB path and returns a struct with
%     P.root       repository folder
%     P.urdf_dir   urdf/g1_description  (URDF + meshes)
%     P.urdf       urdf/g1_description/g1_29dof.urdf
%     P.exports    folder for exported files (poses, CSV, images); created if
%                  missing, with the temp folder as fallback if not writable
%     P.python     python/ folder (MuJoCo viewer)
%     P.version    app version
persistent cache
if ~isempty(cache) && isfolder(cache.root)
    P = cache;
    return
end
here = fileparts(mfilename('fullpath'));            % <root>/src/app
P.root = fileparts(fileparts(here));
addpath(genpath(fullfile(P.root, 'src')));
P.urdf_dir = fullfile(P.root, 'urdf', 'g1_description');
P.urdf = fullfile(P.urdf_dir, 'g1_29dof.urdf');
P.python = fullfile(P.root, 'python');
P.exports = writable_folder(fullfile(P.root, 'exports'));
P.version = '1.0.0';
cache = P;
end

function d = writable_folder(d)
% Folder where files can be written; the temp folder if d is not writable.
ok = isfolder(d);
if ~ok, ok = mkdir(d); end
if ok
    f = fullfile(d, '.write_test');
    fid = fopen(f, 'w');
    if fid < 0
        ok = false;
    else
        fclose(fid);
        delete(f);
    end
end
if ~ok
    d = fullfile(tempdir, 'g1_kinematics_app_exports');
    if ~isfolder(d), mkdir(d); end
end
end
