function q = quantile_semplice(x, p)
%QUANTILE_SEMPLICE  Quantile p di un vettore (interpolazione lineare), senza toolbox.
x = sort(x(:));
n = numel(x);
if n == 1, q = x; return; end
h = 1 + (n - 1) * p;
i = floor(h); i = max(1, min(n - 1, i));
q = x(i) + (h - i) * (x(i + 1) - x(i));
end
