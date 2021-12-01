function [X, time] = CayleyMethod(R)
% The Cayley method.
%
  tic;

  U = [1+R(1,1)+R(2,2)+R(3,3)  R(3,2)-R(2,3)            R(1,3)-R(3,1)           R(2,1)-R(1,2) ;
       R(3,2)-R(2,3)           1+R(1,1)-R(2,2)-R(3,3)   R(1,2)+R(2,1)           R(3,1)+R(1,3) ;
       R(1,3)-R(3,1)           R(1,2)+R(2,1)            1-R(1,1)+R(2,2)-R(3,3)  R(2,3)+R(3,2) ;
       R(2,1)-R(1,2)           R(3,1)+R(1,3)            R(2,3)+R(3,2)           1-R(1,1)-R(2,2)+R(3,3)];

  q = vecnorm(U,2,2);

  [~, index] = max(q);

  switch index
      case 1
          q(1) =              q(1);
          q(2) = sign(U(1,2))*q(2);
          q(3) = sign(U(1,3))*q(3);
          q(4) = sign(U(1,4))*q(4);
      case 2
          q(1) = sign(U(2,1))*q(1);
          q(2) =              q(2);
          q(3) = sign(U(2,3))*q(3);
          q(4) = sign(U(2,4))*q(4);
      case 3
          q(1) = sign(U(3,1))*q(1);
          q(2) = sign(U(3,2))*q(2);
          q(3) =              q(3);
          q(4) = sign(U(3,4))*q(4);
      case 4
          q(1) = sign(U(4,1))*q(1);
          q(2) = sign(U(4,2))*q(2);
          q(3) = sign(U(4,3))*q(3);
          q(4) =              q(4);
  end

  X = Quat2Mat(q);

  time=toc;

end
