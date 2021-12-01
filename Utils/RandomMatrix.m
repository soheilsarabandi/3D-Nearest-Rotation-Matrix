function M = RandomMatrix(errorbound)
% This function returns a 3x3 matrix whose entries are random numbers 
% uniformly distributed in the interval [-errorbound, errorbound].
%

  M = 2*errorbound.*rand(3,3) - errorbound;

end

