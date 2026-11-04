function U = leggi_urdf(file)
%LEGGI_URDF  Legge link e giunti di un file URDF (solo figli diretti di <robot>).
%   U.link(i):   nome, massa, cdm (origine del blocco inertial), rpy_inerziale,
%                inerzia (3x3 nel frame inerziale), sfere (4xm: centro e raggio
%                delle sfere di collisione, nel frame del link), mesh (mesh visive:
%                file, origine, scala)
%   U.giunto(i): nome, tipo, padre, figlio, xyz, rpy, asse, qmin, qmax, vmax, coppia_max
%   La lettura è indipendente da importrobot, che è usato solo per il confronto.

if ~isfile(file), error('leggi_urdf:file', 'File URDF non trovato: %s', file); end
doc = xmlread(file);
rob = doc.getDocumentElement();
U.file = file;
U.nome = char(rob.getAttribute('name'));

% ---------------------------------------------------------------- link
U.link = struct('nome', {}, 'massa', {}, 'cdm', {}, 'rpy_inerziale', {}, ...
                'inerzia', {}, 'sfere', {}, 'mesh', {});
el = figli_tag(rob, 'link');
for i = 1:numel(el)
    e = el{i};
    L = struct();
    L.nome = char(e.getAttribute('name'));
    L.massa = 0;
    L.cdm = zeros(3, 1);
    L.rpy_inerziale = zeros(3, 1);
    L.inerzia = zeros(3);
    in = figli_tag(e, 'inertial');
    if ~isempty(in)
        [L.cdm, L.rpy_inerziale] = leggi_origine(in{1});
        m = figli_tag(in{1}, 'mass');
        if ~isempty(m), L.massa = valore(m{1}, 'value', 0); end
        I = figli_tag(in{1}, 'inertia');
        if ~isempty(I)
            g = @(s) valore(I{1}, s, 0);
            L.inerzia = [g('ixx') g('ixy') g('ixz'); g('ixy') g('iyy') g('iyz'); g('ixz') g('iyz') g('izz')];
        end
    end
    L.sfere = zeros(4, 0);
    co = figli_tag(e, 'collision');
    for c = 1:numel(co)
        geo = figli_tag(co{c}, 'geometry');
        if isempty(geo), continue; end
        sf = figli_tag(geo{1}, 'sphere');
        if ~isempty(sf)
            xyz = leggi_origine(co{c});
            L.sfere(:, end + 1) = [xyz; valore(sf{1}, 'radius', 0)];
        end
    end
    L.mesh = struct('file', {}, 'xyz', {}, 'rpy', {}, 'scala', {});
    vi = figli_tag(e, 'visual');
    for c = 1:numel(vi)
        geo = figli_tag(vi{c}, 'geometry');
        if isempty(geo), continue; end
        me = figli_tag(geo{1}, 'mesh');
        if isempty(me), continue; end
        [xyz, rpy] = leggi_origine(vi{c});
        sc = char(me{1}.getAttribute('scale'));
        if isempty(sc), sc = [1; 1; 1]; else, sc = sscanf(sc, '%f'); end
        L.mesh(end + 1) = struct('file', char(me{1}.getAttribute('filename')), ...
                                 'xyz', xyz, 'rpy', rpy, 'scala', sc);
    end
    U.link(end + 1) = L;
end

% -------------------------------------------------------------- giunti
U.giunto = struct('nome', {}, 'tipo', {}, 'padre', {}, 'figlio', {}, 'xyz', {}, ...
                  'rpy', {}, 'asse', {}, 'qmin', {}, 'qmax', {}, 'vmax', {}, 'coppia_max', {});
el = figli_tag(rob, 'joint');
for i = 1:numel(el)
    e = el{i};
    G = struct();
    G.nome = char(e.getAttribute('name'));
    G.tipo = char(e.getAttribute('type'));
    p = figli_tag(e, 'parent');
    c = figli_tag(e, 'child');
    G.padre  = char(p{1}.getAttribute('link'));
    G.figlio = char(c{1}.getAttribute('link'));
    [G.xyz, G.rpy] = leggi_origine(e);
    a = figli_tag(e, 'axis');
    if isempty(a)
        G.asse = [1; 0; 0];
    else
        G.asse = vettore(a{1}, 'xyz', [1; 0; 0]);
    end
    G.qmin = -Inf; G.qmax = Inf; G.vmax = Inf; G.coppia_max = Inf;
    lim = figli_tag(e, 'limit');
    if ~isempty(lim) && ~strcmp(G.tipo, 'continuous')
        G.qmin = valore(lim{1}, 'lower', -Inf);
        G.qmax = valore(lim{1}, 'upper', Inf);
    end
    if ~isempty(lim)
        G.vmax = valore(lim{1}, 'velocity', Inf);
        G.coppia_max = valore(lim{1}, 'effort', Inf);
    end
    U.giunto(end + 1) = G;
end
end

% ======================================================================
function c = figli_tag(e, tag)
% figli diretti di e con nome tag
c = {};
nodi = e.getChildNodes();
for i = 0:nodi.getLength() - 1
    x = nodi.item(i);
    if x.getNodeType() == 1 && strcmp(char(x.getNodeName()), tag)
        c{end + 1} = x; %#ok<AGROW>
    end
end
end

function [xyz, rpy] = leggi_origine(e)
o = figli_tag(e, 'origin');
if isempty(o)
    xyz = zeros(3, 1); rpy = zeros(3, 1);
else
    xyz = vettore(o{1}, 'xyz', zeros(3, 1));
    rpy = vettore(o{1}, 'rpy', zeros(3, 1));
end
end

function v = valore(e, nome, def)
s = char(e.getAttribute(nome));
if isempty(s), v = def; else, v = str2double(s); end
end

function v = vettore(e, nome, def)
s = char(e.getAttribute(nome));
if isempty(s)
    v = def;
else
    v = sscanf(s, '%f');
    if numel(v) ~= 3, error('leggi_urdf:vettore', 'Attributo %s non valido: "%s"', nome, s); end
end
end
