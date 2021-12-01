function [X, time] = SVDMethodMatlab(R)
% The method using the Matlab SVD implementation.
%
  tic;

  [U,S,V] = svd(R);

  if S(1)*S(2)*S(3)<0 
      X = U*diag([1,1,-1])*V';
  else 
      X = U*V';
  end

  time = toc;

end
