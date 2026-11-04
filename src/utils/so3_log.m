function w = so3_log(R)
%SO3_LOG  Logaritmo su SO(3): vettore w (asse*angolo) tale che so3_exp(w) = R.
c  = max(-1, min(1, (trace(R) - 1) / 2));
th = acos(c);
if th < 1e-8
    w = vee(R - R') / 2;
elseif pi - th < 1e-6
    % vicino a pi: asse dalla colonna dominante di R + I
    [~, i] = max(diag(R));
    E = eye(3);
    a = R(:, i) + E(:, i);
    a = a / norm(a);
    w = th * a;
else
    w = th / (2 * sin(th)) * vee(R - R');
end
end
