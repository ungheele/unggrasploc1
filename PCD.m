function [ normal ] = PCD(ordereddata)

tic;

% DOWNSAMPLING
% data = normdata;
% % % Unnecessary--possibly fix
% ordereddata = zeros(size(normdata));
% ordereddata(indexvalue,:) = data;
data = ordereddata;
% image = reshape(ordereddata, 640, 480, 3);
% image = permute(image, [2, 1, 3]);
% 
% square = 4;
% downsampled = image(2:square:480, 2:square:640,:);
% 
% downsampled = permute(downsampled, [2, 1, 3]);
% dsData = reshape(downsampled, size(downsampled,1)*size(downsampled,2), 3);
% data = dsData;

N = length(data);

%Covariance Matrix Calculation
format = 'Pixel %d\n';
sample = 1000;
eigenvector = zeros(N, 3);

% Pulled this out of the loop for possible speedup
fprintf('Running KNN\n');
nearindices = knnsearch(ordereddata, data, 'k', 100);

fprintf('Beginning main loop\n')
for i=1:N
    if mod(i, 1000) == 0
        fprintf(format, i);
    end
%for i=1:sample
    if sum(data(i,:) == 0) == 3
        continue;
    end
    %Using indexing and kdd tree
    neardata = ordereddata(nearindices(i,:),:);
    ignore = sum((neardata == 0), 2);
    % Note it looks like no zeros get into the nn's
    neardata = neardata(ignore ~= 3, :);

    if sum(ignore) > 0
        keyboard
    end
    
    %Using just -50 to +50
    %if i<50
    %    neardata = data(1:100-i,:);
    %else
    %    neardata = data(i-50:i+50,:);
    %end    
    %Creating weighted values
    % Vectorizing this loop...
%     for j=1:Nnear
%        %fprintf(1,'point %d, weighting point %d\n',i,j);
%        epsilon(j) = exp(-sqrt((neardata(j,1)-data(i,1))^2+...
%            (neardata(j,2)-data(i,2))^2+(neardata(j,3)-data(i,3))^2)^2);
%     end

    Nnear = size(neardata, 1);
    diffs = neardata - repmat(data(i,:), Nnear, 1);
    lens = sqrt(sum(diffs.^2, 2))/0.1;
    epsilon = exp(-lens);

    % Also vectorizing this
%     weightedtrans = zeros(size(neartranspose));
%     for j=1:Nnear
%         %fprintf(1,'point %d multiplypoint %d\n',i,j);
%         weightedtrans(:,j) = neartranspose(:,j).*epsilon(j);
%     end
    weightedtrans = diffs'.*repmat(epsilon', 3, 1);
    
    %Calculating weighted covariance over neighbors
        weightcov = (weightedtrans*diffs)/Nnear;
        [eigvec, eigval] = eig(weightcov);
        eigenval = abs(eigval);
        
        %Finding and storing eigenvector corresponding to smallest
        %eigenvalue
        smallest = min(eigenval(eigenval>0));
        if smallest == eigenval(1,1)
            eigenvector(i,:) = eigvec(:,1)';
%             eigenvector(i,1) = eigvec(1,1);
%             eigenvector(i,2) = eigvec(2,1);
%             eigenvector(i,3) = eigvec(3,1);
        elseif smallest == eigenval(2,2)
            eigenvector(i,:) = eigvec(:,2)';
%             eigenvector(i,1) = eigvec(1,2);
%             eigenvector(i,2) = eigvec(2,2);
%             eigenvector(i,3) = eigvec(3,2);
        else
            eigenvector(i,:) = eigvec(:,3)';
%             eigenvector(i,1) = eigvec(1,3);
%             eigenvector(i,2) = eigvec(2,3);
%             eigenvector(i,3) = eigvec(3,3);
        end
        if eigenvector(i,2) < 0
            eigenvector(i,:) = -eigenvector(i,:);
        end
end
toc

%Plot using line with point and point+eigenvector
% figure;
scatter3(ordereddata(:,1),ordereddata(:,2),ordereddata(:,3),'b.');
hold on
% for i=1:1:N
i = N;
%    hold on
    x = [ordereddata(i,1),(ordereddata(i,1) + eigenvector(i,1))];
    y = [ordereddata(i,2),(ordereddata(i,2) + eigenvector(i,2))];
    z = [ordereddata(i,3),(ordereddata(i,3) + eigenvector(i,3))];
    line(x,y,z, 'Color','r');
normal = eigenvector(i,:);    
end