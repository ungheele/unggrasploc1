function parameterVector = fitQuadric(neighborHood, numberOfNeighborhoodPoints)

    M = zeros(10, 10);

    for i  = 1 : 1 : numberOfNeighborhoodPoints
   
        l = [   
                neighborHood(i, 1) ^ 2;
                neighborHood(i, 2) ^ 2;
                neighborHood(i, 3) ^ 2;
                neighborHood(i, 1) * neighborHood(i, 2);
                neighborHood(i, 1) * neighborHood(i, 3);
                neighborHood(i, 2) * neighborHood(i, 3);
                neighborHood(i, 1);
                neighborHood(i, 2);
                neighborHood(i, 3);
                1
            ];
        
        M = M + l * l';
   
    end

    N = zeros(10, 10);

    for i  = 1: 1 : numberOfNeighborhoodPoints
   
        l1 = [  
                neighborHood(i, 1) * 2;
                0;
                0;
                neighborHood(i, 2);
                neighborHood(i, 3);
                0;
                1;
                0;
                0;
                0
             ];
        
        N = N + l1 * l1';
   
        l2 = [  
                0;
                neighborHood(i, 2) * 2;
                0;
                neighborHood(i, 1);
                0;
                neighborHood(i, 3);
                0;
                1;
                0;
                0
             ];
        
        N = N + l2 * l2';
    
        l3 = [  
                0;
                0;
                neighborHood(i, 3) * 2;
                0;
                neighborHood(i, 1);
                neighborHood(i, 2);
                0;
                0;
                1;
                0
             ];
        
        N = N + l3 * l3';

    end

    [coefficientEigenVectors, coefficientEigenValues] = eig(M, N);

    coefficientEigenValues = diag(coefficientEigenValues);
    coefficientEigenValues = coefficientEigenValues';

    [~, indexMinCoefficientEigenValue] = min(coefficientEigenValues);

    parameterVector = coefficientEigenVectors(:, indexMinCoefficientEigenValue);
    parameterVector = parameterVector';

end