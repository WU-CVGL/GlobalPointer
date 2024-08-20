function [RedundantA, RedundantB] = GetRotationRedundantConstraint()
    RedundantA = cell(21, 1);
    RedundantB = zeros(21, 1);
    
    % -------------- RR^T --------------
    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 1) = 1;
    QuadraticMatrix(2, 2) = 1;
    QuadraticMatrix(3, 3) = 1;
    RedundantA{1} = QuadraticMatrix;
    RedundantB(1) = 1;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(4, 4) = 1;
    QuadraticMatrix(5, 5) = 1;
    QuadraticMatrix(6, 6) = 1;
    RedundantA{2} = QuadraticMatrix;
    RedundantB(2) = 1;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(7, 7) = 1;
    QuadraticMatrix(8, 8) = 1;
    QuadraticMatrix(9, 9) = 1;
    RedundantA{3} = QuadraticMatrix;
    RedundantB(3) = 1;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 4) = 1;
    QuadraticMatrix(2, 5) = 1;
    QuadraticMatrix(3, 6) = 1;
    RedundantA{4} = QuadraticMatrix;
    RedundantB(4) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 7) = 1;
    QuadraticMatrix(2, 8) = 1;
    QuadraticMatrix(3, 9) = 1;
    RedundantA{5} = QuadraticMatrix;
    RedundantB(5) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(4, 7) = 1;
    QuadraticMatrix(5, 8) = 1;
    QuadraticMatrix(6, 9) = 1;
    RedundantA{6} = QuadraticMatrix;
    RedundantB(6) = 0;

    % -------------- R^TR --------------
    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 1) = 1;
    QuadraticMatrix(4, 4) = 1;
    QuadraticMatrix(7, 7) = 1;
    RedundantA{7} = QuadraticMatrix;
    RedundantB(7) = 1;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(2, 2) = 1;
    QuadraticMatrix(5, 5) = 1;
    QuadraticMatrix(8, 8) = 1;
    RedundantA{8} = QuadraticMatrix;
    RedundantB(8) = 1;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(3, 3) = 1;
    QuadraticMatrix(6, 6) = 1;
    QuadraticMatrix(9, 9) = 1;
    RedundantA{9} = QuadraticMatrix;
    RedundantB(9) = 1;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 2) = 1;
    QuadraticMatrix(4, 5) = 1;
    QuadraticMatrix(7, 8) = 1;
    RedundantA{10} = QuadraticMatrix;
    RedundantB(10) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 3) = 1;
    QuadraticMatrix(4, 6) = 1;
    QuadraticMatrix(7, 9) = 1;
    RedundantA{11} = QuadraticMatrix;
    RedundantB(11) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(2, 3) = 1;
    QuadraticMatrix(5, 6) = 1;
    QuadraticMatrix(8, 9) = 1;
    RedundantA{12} = QuadraticMatrix;
    RedundantB(12) = 0;


    % -------------- R times R --------------
    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(4, 8) = 1;
    QuadraticMatrix(5, 7) = -1;
    QuadraticMatrix(3, 10) = -1;
    RedundantA{13} = QuadraticMatrix;
    RedundantB(13) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(2, 7) = 1;
    QuadraticMatrix(1, 8) = -1;
    QuadraticMatrix(6, 10) = -1;
    RedundantA{14} = QuadraticMatrix;
    RedundantB(14) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 5) = 1;
    QuadraticMatrix(2, 4) = -1;
    QuadraticMatrix(9, 10) = -1;
    RedundantA{15} = QuadraticMatrix;
    RedundantB(15) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(5, 9) = 1;
    QuadraticMatrix(6, 8) = -1;
    QuadraticMatrix(1, 10) = -1;
    RedundantA{16} = QuadraticMatrix;
    RedundantB(16) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(3, 8) = 1;
    QuadraticMatrix(2, 9) = -1;
    QuadraticMatrix(4, 10) = -1;
    RedundantA{17} = QuadraticMatrix;
    RedundantB(17) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(2, 6) = 1;
    QuadraticMatrix(3, 5) = -1;
    QuadraticMatrix(7, 10) = -1;
    RedundantA{18} = QuadraticMatrix;
    RedundantB(18) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(6, 7) = 1;
    QuadraticMatrix(4, 9) = -1;
    QuadraticMatrix(2, 10) = -1;
    RedundantA{19} = QuadraticMatrix;
    RedundantB(19) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(1, 9) = 1;
    QuadraticMatrix(3, 7) = -1;
    QuadraticMatrix(5, 10) = -1;
    RedundantA{20} = QuadraticMatrix;
    RedundantB(20) = 0;

    QuadraticMatrix = zeros(10, 10);
    QuadraticMatrix(3, 4) = 1;
    QuadraticMatrix(1, 6) = -1;
    QuadraticMatrix(8, 10) = -1;
    RedundantA{21} = QuadraticMatrix;
    RedundantB(21) = 0;



    for i = 1:21
        raw_matrix = RedundantA{i};
        zero_matrix = zeros(13, 13);
        zero_matrix(1:9, 1:9) = raw_matrix(1:9, 1:9);
        zero_matrix(13, 1:9) = raw_matrix(10, 1:9);
        zero_matrix(1:9, 13) = raw_matrix(1:9, 10);
        zero_matrix(13, 13) = raw_matrix(10, 10);
        RedundantA{i} = zero_matrix;
    end

end