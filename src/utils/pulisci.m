function x = pulisci(x, tol)
%PULISCI  Azzera gli elementi numerici con valore assoluto sotto tol (default 1e-14).
if nargin < 2, tol = 1e-14; end
x(abs(x) < tol) = 0;
end
