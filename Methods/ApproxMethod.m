function [X, time] = ApproxMethod(R)
% The approximated method.
%
  tic;

  U = [1+R(1,1)+R(2,2)+R(3,3)  R(3,2)-R(2,3)            R(1,3)-R(3,1)           R(2,1)-R(1,2) ;
       R(3,2)-R(2,3)           1+R(1,1)-R(2,2)-R(3,3)   R(1,2)+R(2,1)           R(3,1)+R(1,3) ;
       R(1,3)-R(3,1)           R(1,2)+R(2,1)            1-R(1,1)+R(2,2)-R(3,3)  R(2,3)+R(3,2) ;
       R(2,1)-R(1,2)           R(3,1)+R(1,3)            R(2,3)+R(3,2)           1-R(1,1)-R(2,2)+R(3,3)];

  [~,index] = max(sum(U.^2,2));

  q1 = sign(dot(U(index,:),U(1,:)))*U(1,:);
  q2 = sign(dot(U(index,:),U(2,:)))*U(2,:);
  q3 = sign(dot(U(index,:),U(3,:)))*U(3,:);
  q4 = sign(dot(U(index,:),U(4,:)))*U(4,:);

  X = Quat2Mat(q1+q2+q3+q4);

  time=toc;

end
