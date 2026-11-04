function [R, p] = posa_frame(mod, K, f)
%POSA_FRAME  Orientamento e origine di un frame (nome o indice) nel mondo.
f  = trova_frame(mod, f);
fr = mod.frame(f);
Rc = fr.R; pc = fr.p;
if K.simbolico
    Rc = costante_sym(Rc);
    pc = costante_sym(pc);
end
Rk = K.R{fr.corpo + 1};
R  = Rk * Rc;
p  = K.p{fr.corpo + 1} + Rk * pc;
end
