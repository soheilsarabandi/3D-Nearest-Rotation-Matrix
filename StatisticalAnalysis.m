function [error,time]=StatisticalAnalysis()
% Generates random noisy rotation matrices with different levels of
% noise and evaluates the following methods described in the paper:
%   - SVD
%   - Exact
%   - Approximated
%   - Cayley
%

  samples = 10000;
  samples_error = 100;

  low_noise = 0;
  high_noise = 0.5;
  
  ero = zeros(8,samples,samples_error);
  error = zeros(6,8,samples_error);
  tm = zeros(4,samples,samples_error);
  time = zeros(1,4);
  
  errorbound = linspace(low_noise, high_noise, samples_error); 

  for j=1:samples_error % level of error
      fprintf('Iteration %u of %u\n',j,samples_error);
      for i = 1:samples % sample for the given level of error
          R1 = Quat2Mat(RandomQuaternion)+ RandomMatrix(errorbound(j));
          [M1, tm(1,i,j)] = SVDMethodMatlab(R1);
          [M2, tm(2,i,j)] = ExactMethod(R1);
          [M3, tm(3,i,j)] = ApproxMethod(R1);
          [M4, tm(4,i,j)] = CayleyMethod(R1);
          ero(1,i,j) = norm(R1-M1,'fro');
          ero(2,i,j) = norm(R1-M2,'fro');
          ero(3,i,j) = norm(R1-M3,'fro');
          ero(4,i,j) = norm(R1-M4,'fro');
          ero(5,i,j) = norm(M1'*M1-eye(3,3),'fro');
          ero(6,i,j) = norm(M2'*M2-eye(3,3),'fro');
          ero(7,i,j) = norm(M3'*M3-eye(3,3),'fro');
          ero(8,i,j) = norm(M4'*M4-eye(3,3),'fro');
      end
  end

  for i=1:4
    time(i)=mean(mean(tm(i,:,:)));
  end
  
  for j=1:samples_error
      for i=1:4 
          error(1,i,j) = mean(ero(i,:,j));
          error(2,i,j) = min(ero(i,:,j));
          error(3,i,j) = max(ero(i,:,j));
      end
  end

  for j=1:samples_error
      for i=5:8
          error(4,i,j) = mean(ero(i,:,j));
          error(5,i,j) = min(ero(i,:,j));
          error(6,i,j) = max(ero(i,:,j));
      end
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  figure(1); % Mean distance error
  plot(errorbound, squeeze(error(1,1,:)),'k','LineWidth',2); % SVD
  hold on;
  plot(errorbound, squeeze(error(1,2,:)),'r','LineWidth',1); % Exact
  plot(errorbound, squeeze(error(1,3,:)),'g');               % Approx 
  plot(errorbound, squeeze(error(1,4,:)),'b');               % Cayley 
  ylabel('Mean Frobenius norm');
  xlabel('Input error');
  legend('SVD','Exact method', ...
               'Approx method 1', ...
               'Cayley method 2', ...
               'Location','northwest');
  grid on;

  figure(2); % Minimum distance error
  plot(errorbound, squeeze(error(2,1,:)),'k','LineWidth',2); % SVD
  hold on;
  plot(errorbound, squeeze(error(2,2,:)),'r','LineWidth',1); % Exact
  plot(errorbound, squeeze(error(2,3,:)),'g');               % Approx
  plot(errorbound, squeeze(error(2,4,:)),'b');               % Cayley
  ylabel('Minimum Frobenius norm');
  xlabel('Input error');
  legend('SVD','Exact method',...
               'Approx method 1',...
               'Cayley method 2',...
               'Location','northwest');
  grid on;

  figure(3); % Maximum distance error
  plot(errorbound, squeeze(error(3,1,:)),'k','LineWidth',2); % SVD
  hold on;
  plot(errorbound, squeeze(error(3,2,:)),'r','LineWidth',1); % Exact
  plot(errorbound, squeeze(error(3,3,:)),'g');               % Approx
  plot(errorbound, squeeze(error(3,4,:)),'b');               % Cayley
  ylabel('Maximum Frobenius norm');
  xlabel('Input error');
  legend('SVD','Exact method',...
               'Approx method 1',...
               'Cayley method 2',...
               'Location','northwest');
  grid on;

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  figure(4); % Mean orthogonality error
  semilogy(errorbound, squeeze(error(4,5,:)),'k',...  % SVD
           errorbound, squeeze(error(4,6,:)),'r',...  % Exact
           errorbound, squeeze(error(4,7,:)),'g',...  % Approx 
           errorbound, squeeze(error(4,8,:)),'b');    % Cayley
  ylabel('Mean orthogonality error');
  xlabel('Input error');
  legend('SVD','Exact method',...
               'Approx method 1',...
               'Cayley method 2',...
               'Location','northwest');
  grid on;

  figure(5); % Min orthogonality error
  semilogy(errorbound, squeeze(error(5,5,:)),'k',...  % SVD
           errorbound, squeeze(error(5,6,:)),'r',...  % Exact
           errorbound, squeeze(error(5,7,:)),'g',...  % Approx
           errorbound, squeeze(error(5,8,:)),'b');    % Cayley
  ylabel('Minimum orthogonality error');
  xlabel('Input error');
  legend('SVD','Exact method',...
               'Approx method 1',...
               'Cayley method 2',...
               'Location','northwest');
  grid on;

  figure(6); % Max orthogonality error
  semilogy(errorbound, squeeze(error(6,5,:)),'k',...  % SVD
           errorbound, squeeze(error(6,6,:)),'r',...  % Exact
           errorbound, squeeze(error(6,7,:)),'g',...  % Approrx
           errorbound, squeeze(error(6,8,:)),'b');    % Cayley
  ylabel('Maximum orthogonality error');
  xlabel('Input error');
  legend('SVD','Exact method',...
               'Approx method 1',...
               'Cayley method 2',...
               'Location','northwest');
  grid on;

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end


