% This function is part of the GlobalPointer method as described in [1]. When you
% use this code, you are required to cite [1].
% 
% [1] GlobalPointer: Large-Scale Plane Adjustment with Bi-Convex Relaxation
% Author: B. Liao, Z. Zhao, L. Chen, H. Li, D. Cremers, P. Liu.
% European Conference on Computer Vision 2024 (ECCV 2024)
%
% 
% Author & Copyright (C) 2024: Bangyan Liao (liaobangyan[at]westlake[dot]edu[dot]cn)
%                              Zhenjun Zhao (ericzzj89[at]gmail[dot]com)
%                              Peidong Liu (liupeidong[at]westlake[dot]edu[dot]cn)

function [plane_estimate_struct,plane_estimate_matrix,optimal_plane_vec] = UpdatePlaneResults(X_list, param)

    optimal_plane_vec = ones(1, param.plane_num);

    for plane_i = 1:param.plane_num
        [~, S, V] = svd(X_list{1, plane_i});
        A = S*V';
        x_solution = A(1, :);
        x_solution = x_solution / norm(x_solution(1, 1:3));

        plane_estimate_struct.normal_vector_list{1, plane_i} = x_solution(1:3)';
        plane_estimate_struct.q_list{1, plane_i} = x_solution(4);
    end

    plane_estimate_matrix = zeros(4, param.plane_num);
    
    for plane_i = 1:param.plane_num
        plane_estimate_matrix(:, plane_i) = [plane_estimate_struct.normal_vector_list{plane_i}; plane_estimate_struct.q_list{plane_i}];
    end

end