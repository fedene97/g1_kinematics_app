function S = costante_sym(x)
%COSTANTE_SYM  Converte costanti numeriche in sym senza razionali esatti.
%   Gli interi (0, 1, -1, ...) restano esatti; gli altri valori diventano numeri
%   vpa a 16 cifre. Convertire i double in razionali (comportamento di default
%   di MATLAB) fa esplodere le espressioni.
x = pulisci(double(x), 1e-14);
S = vpa(x, 16);
intero = (x == round(x));
if any(intero(:))
    S(intero) = sym(x(intero));
end
end
