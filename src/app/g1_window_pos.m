function pos = g1_window_pos(w, h)
%G1_WINDOW_POS  Centered window position of size w x h, shrunk to fit the screen.
sz = get(groot, 'ScreenSize');                       % [1 1 width height]
w = min(w, sz(3) - 60);
h = min(h, sz(4) - 120);
pos = [max(10, round((sz(3) - w) / 2)), max(40, round((sz(4) - h) / 2)), w, h];
end
