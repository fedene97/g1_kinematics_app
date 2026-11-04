function controlli_vista(parent, callback, ax, cartella)
%CONTROLLI_VISTA  Display panel: what to show, camera presets, image export.
%   controlli_vista(parent, callback, ax, cartella)
%   callback(vis) receives the visibility struct (see vista_g1_visibilita);
%   ax: the axes with the robot (camera presets, image export);
%   cartella: default folder of the "Save image" dialog.
gl = uigridlayout(parent, [12 1]);
gl.RowHeight = {26, 26, 26, 26, 26, 26, 26, 22, 40, 22, 30, 32};
voci = {'scheletro', 'Skeleton', true; 'mesh', 'CAD meshes (STL)', true; 'capsule', 'Collision capsules', false;
        'tcp', 'TCP frames and CoM', true; 'gambe', 'Legs', true; 'tronco', 'Pelvis, waist, torso, head', true;
        'braccia', 'Arms', true};
cb = struct();
for i = 1:size(voci, 1)
    cb.(voci{i, 1}) = uicheckbox(gl, 'Text', voci{i, 2}, 'Value', voci{i, 3}, ...
                                 'ValueChangedFcn', @(~, ~) invia());
end
uilabel(gl, 'Text', 'Mesh opacity');
sl = uislider(gl, 'Limits', [0.1 1], 'Value', 1, 'ValueChangedFcn', @(~, ~) invia());
uilabel(gl, 'Text', 'Camera');
rv = uigridlayout(gl, [1 4]);
rv.Padding = [0 0 0 0]; rv.ColumnSpacing = 4;
viste = {'Front', 90, 0; 'Side', 0, 0; 'Top', 0, 90; '3D', 40, 15};
for i = 1:size(viste, 1)
    az = viste{i, 2}; el = viste{i, 3};
    uibutton(rv, 'Text', viste{i, 1}, 'ButtonPushedFcn', @(~, ~) view(ax, az, el));
end
uibutton(gl, 'Text', 'Save image (PNG)', 'ButtonPushedFcn', @(~, ~) salva_immagine());

    function invia()
        vis = struct();
        for j = 1:size(voci, 1), vis.(voci{j, 1}) = cb.(voci{j, 1}).Value; end
        vis.alfa_mesh = sl.Value;
        callback(vis);
    end
    function salva_immagine()
        fig = ancestor(parent, 'figure');
        [f, d] = uiputfile({'*.png', 'PNG image (*.png)'}, 'Save image', fullfile(cartella, 'g1_view.png'));
        if isequal(f, 0), return; end
        try
            exportgraphics(ax, fullfile(d, f), 'Resolution', 200, 'BackgroundColor', 'white');
        catch ME
            uialert(fig, ME.message, 'Could not save the image');
        end
    end
end
