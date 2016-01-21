clear all
close all

%% extract data from pointcloud

pointCloudsRaw = load('now.pcd');

numberOfPointsRaw = length(pointCloudsRaw);

indices = zeros(numberOfPointsRaw, 1);

pointClouds = zeros(numberOfPointsRaw, 3);

numberOfPoints = 0;
% for i = 1 : numberOfPointsRaw
% for i = 80000: numberOfPointsRaw-90000
for i = 1: numberOfPointsRaw
    
    if isfinite(pointCloudsRaw(i, 1)) && isfinite(pointCloudsRaw(i, 2)) && isfinite(pointCloudsRaw(i, 3))

        numberOfPoints = numberOfPoints + 1;

        indices(numberOfPoints, 1) = i;

        pointClouds(numberOfPoints, :) = pointCloudsRaw(i, :);

    end
    
    
        

end






indices = indices(1 : numberOfPoints, :);

pointClouds = pointClouds(1 : numberOfPoints, :);

% pointClouds = pointClouds(50000 :numberOfPoints, :);

% numberOfPoints=length(pointClouds);
plot3(pointClouds(:, 1), pointClouds(:, 2), pointClouds(:, 3), 'k.')
hold on;
    

%% sampling
numberOfNeighborhoodCentroids = 2000;

[neighborhoodCentroids, neighborhoodCentroidsIndices] = datasample(pointClouds, numberOfNeighborhoodCentroids);
neighborhoodCentroidsIndices = neighborhoodCentroidsIndices';
neighborhoodCentroids = [neighborhoodCentroids neighborhoodCentroidsIndices];
neighborhoods = struct();

for i = 1 : numberOfNeighborhoodCentroids

    singleNeighborhoodPoints = zeros(numberOfPoints, 3);
    numberOfSingleNeighborhoodPoints = 0;
    
    for j = 1 : numberOfPoints
        
        xNormSquare = (neighborhoodCentroids(i, 1) - pointClouds(j, 1)) ^ 2;
        yNormSquare = (neighborhoodCentroids(i, 2) - pointClouds(j, 2)) ^ 2;
        zNormSquare = (neighborhoodCentroids(i, 3) - pointClouds(j, 3)) ^ 2;
        
        distance = sqrt(xNormSquare +  yNormSquare + zNormSquare);

        if  distance < 0.05
            
            numberOfSingleNeighborhoodPoints = numberOfSingleNeighborhoodPoints + 1;
            
            singleNeighborhoodPoints(numberOfSingleNeighborhoodPoints, :) = pointClouds(j, :);
            
        end
        
    end
    
    l = ['neighborhood' num2str(i)];
    
    singleNeighborhoodPoints = singleNeighborhoodPoints(1 : numberOfSingleNeighborhoodPoints, :);
    
    neighborhoods.(l) = singleNeighborhoodPoints;

end

%plot3(neighborhoodCentroids(:, 1), neighborhoodCentroids(:, 2), neighborhoodCentroids(:, 3), 'ro')
%hold on;
    

%iterate through every sampled neighborhood

curvatures = zeros(numberOfNeighborhoodCentroids, 1);
normals = zeros(numberOfNeighborhoodCentroids, 3);
principalAxes = zeros(numberOfNeighborhoodCentroids, 3);
principalDirections = zeros(numberOfNeighborhoodCentroids, 3);

for i = 1 : numberOfNeighborhoodCentroids

    neighborhood = neighborhoods.(['neighborhood' num2str(i)]);
    numberOfNeighborhoodPoints = length(neighborhood);

    %plot3(neighborhood(:, 1), neighborhood(:, 2), neighborhood(:, 3), 'b.');
    %hold on

    parameterVector = fitQuadric(neighborhood, numberOfNeighborhoodPoints);

    % add output of median value to use if

    [curvature ,normal, principalAxis, principalDirection] = estimateMedianCurvature(neighborhood, numberOfNeighborhoodPoints, parameterVector);

    curvatures(i) = curvature;
    normals(i, :) = normal;
    principalAxes(i, :) = principalAxis;
    principalDirections(i, :) = principalDirection;
    
end

%filter out some neighborhoods below some threshhold
filteredNeighborhoodsIndices = zeros(numberOfNeighborhoodCentroids, 1);
numberOfFilteredNeighborhoods = 0;
for i = 1 : numberOfNeighborhoodCentroids
   
    if curvatures(i) > 2
%       if curvatures(i) > 0  
        numberOfFilteredNeighborhoods = numberOfFilteredNeighborhoods + 1;
        
        filteredNeighborhoodsIndices(numberOfFilteredNeighborhoods) = i;
        
    end
    
end

filteredNeighborhoodsIndices = filteredNeighborhoodsIndices(1 : numberOfFilteredNeighborhoods);

circlesInfo = zeros(numberOfFilteredNeighborhoods, 3);
circleCenterInfo = zeros(numberOfFilteredNeighborhoods, 3);
shellCentroids = zeros(numberOfFilteredNeighborhoods, 3);
shellExtents = zeros(numberOfFilteredNeighborhoods, 1);

for i = 1 : numberOfFilteredNeighborhoods

    neighborhood = neighborhoods.(['neighborhood' num2str(filteredNeighborhoodsIndices(i))]);
    numberOfNeighborhoodPoints = length(neighborhood);
    
    normal = normals(filteredNeighborhoodsIndices(i), :);
    principalAxis = principalAxes(filteredNeighborhoodsIndices(i), :);
    neighborhoodCentroid = neighborhoodCentroids(filteredNeighborhoodsIndices(i), :);
    
    [circleCenterX, circleCenterY, circleRadius, shellCentroid, shellExtent, circleCenter] = fitCylinder(neighborhood, numberOfNeighborhoodPoints, normal, principalAxis, neighborhoodCentroid);
    
    circlesInfo(i, :) = [circleCenterX, circleCenterY, circleRadius];
    circleCenterInfo(i, :) = circleCenter;
    shellCentroids(i, :) = shellCentroid;
    shellExtents(i) = shellExtent;

end

%filter out the neighborhoods with a gap to let the robot hand fit in
gapFilteredNeighborhoodsIndices = zeros(numberOfFilteredNeighborhoods, 1);
numberOfGapFilteredNeighborhoodsIndices = 0;
for i = 1 : numberOfFilteredNeighborhoods

    neighborhood = neighborhoods.(['neighborhood' num2str(filteredNeighborhoodsIndices(i))]);
    numberOfNeighborhoodPoints = length(neighborhood);
    
    principalAxis = principalAxes(filteredNeighborhoodsIndices(i), :);
    shellCentroid = shellCentroids(i, :);
    shellExtent = shellExtents(i, :);
    circleRadius = circlesInfo(i, 3);
    
    if hasClearance(neighborhood, numberOfNeighborhoodPoints, principalAxis, shellCentroid, circleRadius, shellExtent);
        
        numberOfGapFilteredNeighborhoodsIndices = numberOfGapFilteredNeighborhoodsIndices + 1;
        
        gapFilteredNeighborhoodsIndices(numberOfGapFilteredNeighborhoodsIndices) = i;
        
    end
    
end

gapFilteredNeighborhoodsIndices = gapFilteredNeighborhoodsIndices(1 : numberOfGapFilteredNeighborhoodsIndices);

for i=1 : numberOfGapFilteredNeighborhoodsIndices
    
    neighborhood = neighborhoods.(['neighborhood' num2str(filteredNeighborhoodsIndices(gapFilteredNeighborhoodsIndices(i)))]);
    numberOfNeighborhoodPoints = length(neighborhood);
    
    plot3(neighborhood(:, 1), neighborhood(:, 2), neighborhood(:, 3), 'g.');
    hold on;
    
end


numberOfgap=length(gapFilteredNeighborhoodsIndices);

neighborhoodTesting=struct();
normalTesting=zeros(numberOfgap,3);
principalAxisTesting=zeros(numberOfgap,3);
neighborhoodCentroidTesting=zeros(numberOfgap,3);

for i = 1:numberOfgap

l = ['neighborhoodTesting' num2str(i)];    
    
neighborhoodTesting.(l) = neighborhoods.(['neighborhood' num2str(filteredNeighborhoodsIndices(gapFilteredNeighborhoodsIndices(i)))]);
normalTesting(i,:) = normals(filteredNeighborhoodsIndices(gapFilteredNeighborhoodsIndices(i)), 1:3);
principalAxisTesting(i,:) = principalAxes(filteredNeighborhoodsIndices(gapFilteredNeighborhoodsIndices(i)), 1:3);
% principalDirection_1= principalDirections(filteredNeighborhoodsIndices(gapFilteredNeighborhoodsIndices(i)), :);
neighborhoodCentroidTesting(i,:)=neighborhoodCentroids(filteredNeighborhoodsIndices(gapFilteredNeighborhoodsIndices(i)), 1:3);






end

mini=zeros(numberOfgap,3);
for i=1:numberOfgap
    l = ['neighborhoodTesting' num2str(i)]; 
    [minZ,indexMin]=min(abs(neighborhoodTesting.(l)(:,3)));
    mini(i,:)=neighborhoodTesting.(l)(indexMin,:);
    
end
[finalMini,index]=min(mini(:,3));
finalMini_neighborhood=mini(index,:);
neighborhoodTesting.(['neighborhoodTesting' num2str(index)])



save('principalAxisTesting.mat','principalAxisTesting');
save('normalTesting.mat','normalTesting');
save('neighborhoodCentroidTesting.mat','neighborhoodCentroidTesting');


quiver3(neighborhoodCentroid(1, 1), neighborhoodCentroid(1, 2), neighborhoodCentroid(1, 3), normalTesting(1,1), normalTesting(1,2), normalTesting(1,3), 'b');
quiver3(neighborhoodCentroid(1, 1), neighborhoodCentroid(1, 2), neighborhoodCentroid(1, 3), principalAxisTesting(1,1), principalAxisTesting(1,2), principalAxisTesting(1,3),'y');
%  quiver3(neighborHood(indexMedianCurvature, 1), neighborHood(indexMedianCurvature, 2), neighborHood(indexMedianCurvature, 3), principalDirectionDisplay(1), principalDirectionDisplay(2), principalDirectionDisplay(3), 'g');
hold on;

