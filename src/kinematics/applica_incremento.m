function x = applica_incremento(x, d)
%APPLICA_INCREMENTO  Aggiorna la configurazione con un incremento d = [dv; dw; dq] (35x1).
%   Stessa convenzione delle velocità: dv e dw nel frame della base.
%     pb <- pb + Rb*dv,   Rb <- Rb*exp(S(dw)),   qj <- qj + dq
d = d(:);
x.pb = x.pb + x.Rb * d(1:3);
x.Rb = x.Rb * so3_exp(d(4:6));
[U, ~, W] = svd(x.Rb);                   % riortonormalizza: evita la deriva numerica
x.Rb = U * W';
x.qj = x.qj + d(7:end);
end
