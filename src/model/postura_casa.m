function qj = postura_casa(mod)
%POSTURA_CASA  Postura di partenza non singolare: ginocchi e gomiti piegati.
%   Con qj = 0 le gambe sono quasi tese (vicino alla singolarità della gamba):
%   è un cattivo punto di partenza per la cinematica inversa.
qj = zeros(mod.n, 1);
imposta = {'left_hip_pitch_joint', -0.2; 'right_hip_pitch_joint', -0.2;
           'left_knee_joint', 0.4;       'right_knee_joint', 0.4;
           'left_ankle_pitch_joint', -0.2; 'right_ankle_pitch_joint', -0.2;
           'left_shoulder_roll_joint', 0.2; 'right_shoulder_roll_joint', -0.2;
           'left_elbow_joint', 0.6;      'right_elbow_joint', 0.6};
for i = 1:size(imposta, 1)
    k = find(strcmp(mod.giunti, imposta{i, 1}), 1);
    if ~isempty(k), qj(k) = min(max(imposta{i, 2}, mod.qmin(k)), mod.qmax(k)); end
end
end
