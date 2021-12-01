function [X, time] = ExactMethod(R)
% The exact method.
%
  tic;

  A  = R'*R;

  m = trace(A)/3;
  Q = A-m*eye(3);
  q = det(Q)/2;
  p = sum(sum(Q.^2))/6;
  sp = sqrt(p);

  theta = atan2(sqrt(abs(p^3-q^2)), q)/3;

  ctheta = cos(theta);
  stheta = sin(theta);

  x1 = abs(m + 2*sp*ctheta);
  x2 = abs(m - sp*(ctheta+sqrt(3)*stheta));
  x3 = abs(m - sp*(ctheta-sqrt(3)*stheta));

  % Computation of the square root of A

  a2 = sqrt(x1)+sqrt(x2)+sqrt(x3);
  a1 = sqrt(x1*x2)+sqrt(x1*x3)+sqrt(x3*x2);
  a0 = sqrt(x1*x2*x3);

  dem = a0*(a2*a1-a0);

  b2 = a2/dem;
  b1 = (a0+a2*(a2^2 -2*a1))/dem;
  b0 = (a2*a1^2 -a0*(a2^2 +a1))/dem;

  U = b2*A*A -b1*A +b0*eye(3);

  X= R*U;

  time = toc;

end