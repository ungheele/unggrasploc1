function handles = searchHandles(pointClouds, numberOfPoints, shells, numberOfShells, principalAxes, centroids, radii)

    alignmentRuns = 3;
    alignmentMinInliers = 10;
    useOcclusionFilter = true;
    maxNumberInFront = 20;
    targetRadius = 0.08;
    radiusError = 0.013;

    handles = zeros(numberOfShells, numberOfShells);
    numberOfHandles = 0;

    % find colinear sets of cylinders
    if alignmentRuns > 0

        % linear search
        i = 0;
        while (i <= alignmentRuns) && (numberOfShells > 0)
            
            [inliersMaxSet, numberOfInliers] = findBestColinearSet(numberOfShells, principalAxes, centroids, radii);

            if numberOfInliers >= alignmentMinInliers

                handle = zeros(1, numberOfShells);
                numberOfHandle = 0;
                
                for j = 1: 1 : numberOfInliers
        
                    index = inliersMaxSet(j);
                    
                    numberOfHandle = numberOfHandle + 1;
                    handle(numberOfHandle) = shells(index);

                end

                if useOcclusionFilter
                    
                    maxNumberOfOccluded = floor(numberOfHandle * 0.5);
                    numberOfOccluded = 0;
                    isOccluded = false;
                    
                    for j = 1 : 1 : numberOfHandle
                        
                        centroidIndex = centroids(j);
                        radius = 1.5 * targetRadius + radiusError;
                        
                        numberInFront = numberInFront(pointClouds, numberOfPoints, centroidIndex, radius);
                        
                        if (numberInFront > maxNumberInFront)
            
                            numberOfOccluded = numberOfOccluded + 1;
                        
                            if (numberOfOccluded > maxNumberOfOccluded)
              
                                isOccluded = true;
                                break;
              
                            end
                            
                        end
                        
                    end

                    if ~isOccluded
              
                        numberOfHandles = numberOfHandles + 1;
                        handles(numberOfHandles, :) = handle;
              
                    end
        
                else
        
                        numberOfHandles = numberOfHandles + 1;
                        handles(numberOfHandles, :) = handle;
          
                end

            % do not check for occlusions
            else
                
                break;

            end 
  
        end
    
    end

end

function numberInFront = numberInFront(pointClouds, numberOfPoints, centroidIndex, radius)

    center = pointClouds(centroidIndex, :);
    distanceCenter = norm(center);
    theta = atan(radius / distanceCenter);
    centerUnit = center / distanceCenter;

    numberInFront = 0;

    for i = 1 : 1 : numberOfPoints
    
        point = pointClouds(i, :);
        pointNorm = norm(point);
        pointUnit = point / pointNorm;

        if (abs(dot(pointUnit, centerUnit)) < cos(theta))
        
            continue;
      
        end

        if (pointNorm < distanCenter - radius)
        
            numberInFront = numberInFront + 1;

        end

    end

end

function [inliersMaxSet, numberOfInliers] = findBestColinearSet(numberOfShells, principalAxes, centroids, radii)

    maxInliers = 0;
    orientRadius2 = 0.01;
    distanceRadius2 = 0.0004;
    alignmentRadiusRadius = 0.003;

    inliers = zeros(1, numberOfShells);
    numberOfInliers = 0;

    for i = 1: 1 :numberOfShells

        principalAxis1 = principalAxes(i, :);
        centroid1 = centroids(i, :);
        radius1 = radii(i, :);

        for j = 1 : 1 : numberOfShells
    
            identityMatrix = eye(3, 3);
            principalAxis2 = principalAxes(i, :);
            centroid2 = centroids(i, :);
            radius2 = radii(i, :);
        
            distanceToOrientVector = (identityMatrix - principalAxis1' * principalAxis1) * principalAxis2';
            distanceToOrient = sum(distanceToOrientVector.^2);
            distanceToPrincipalAxisVector = (identityMatrix - principalAxis1' * principalAxis1) * (centroid2 - centroid1);
            distanceToPrincipalAxis = sum(distanceToPrincipalAxisVector.^2);
            distanceToRadius = abs(radius2 - radius1);

            if ((distanceToOrient < orientRadius2) && (distanceToPrincipalAxis < distanceRadius2) && (distanceToRadius < alignmentRadiusRadius))
          
                numberOfInliers = numberOfInliers + 1;
                inliers(numberOfInliers) = j;
      
            end
      
        end
    
        inliers = inliers(1 : numberOfInliers);

        if (numberOfInliers > maxInliers)
        
            maxInliers = numberOfInliers;
            inliersMaxSet = inliers;
     
        end
    
    end

end