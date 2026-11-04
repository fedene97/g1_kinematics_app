function g1_app(mode)
%G1_APP  Launcher for the Unitree G1 kinematics apps.
%   g1_app        opens a small launcher window
%   g1_app fk     forward kinematics: one slider per joint -> poses of the TCPs
%   g1_app ik     inverse kinematics: move hands / feet / pelvis -> joint angles
%   g1_app test   quick self-test of the non-graphical parts
%   Run it from the repository folder, or add that folder to the path first:
%       addpath('/path/to/g1_kinematics_app'); g1_app
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'src', 'app'));
addpath(fullfile(here, 'tests'));
P = g1_setup();
if nargin < 1, mode = ''; end
switch lower(char(mode))
    case 'fk'
        g1_gui();
    case 'ik'
        g1_gui_ik();
    case 'test'
        g1_smoke_test();
    case ''
        launcher(P);
    otherwise
        error('g1_app:mode', 'Unknown mode "%s". Use g1_app, g1_app fk, g1_app ik or g1_app test.', char(mode));
end
end

function launcher(P)
fig = uifigure('Name', 'Unitree G1 kinematics', 'Position', g1_window_pos(460, 340), 'Resize', 'off');
g = uigridlayout(fig, [6 1]);
g.RowHeight = {44, 24, 58, 58, 26, '1x'}; g.Padding = [30 20 30 12]; g.RowSpacing = 10;
uilabel(g, 'Text', 'Unitree G1 kinematics', 'FontSize', 24, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
uilabel(g, 'Text', '29 joints + floating base (35 DoF)', 'FontSize', 13, 'HorizontalAlignment', 'center', ...
        'FontColor', [0.35 0.35 0.35]);
b1 = uibutton(g, 'Text', 'Forward kinematics', 'FontSize', 16, 'ButtonPushedFcn', @(~, ~) lancia(@g1_gui));
b2 = uibutton(g, 'Text', 'Inverse kinematics', 'FontSize', 16, 'ButtonPushedFcn', @(~, ~) lancia(@g1_gui_ik));
st = uilabel(g, 'Text', '', 'HorizontalAlignment', 'center', 'FontColor', [0.35 0.35 0.35]);
uilabel(g, 'Text', ['v' P.version ' - sliders -> poses  |  TCP targets -> joint angles'], 'FontSize', 11, ...
        'HorizontalAlignment', 'center', 'FontColor', [0.5 0.5 0.5]);
    function lancia(f)
        b1.Enable = 'off'; b2.Enable = 'off';
        st.Text = 'Loading the model and the CAD meshes...'; st.FontColor = [0.35 0.35 0.35];
        drawnow;
        try
            f();
            st.Text = 'Opened. You can launch another one.';
        catch ME
            st.Text = '';
            uialert(fig, ME.message, 'Could not start the app');
        end
        b1.Enable = 'on'; b2.Enable = 'on';
    end
end
