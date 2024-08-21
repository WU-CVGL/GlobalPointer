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

function total_error = TotalErrorBset(pose, B_sets, plane, param)

    total_error = 0;
    point_num = 0;

    lidar_pose_num = size(B_sets, 1);
    plane_num = size(B_sets, 2);
    
    for lidar_pose_i = 1:lidar_pose_num
        R = pose(lidar_pose_i).R;
        t = pose(lidar_pose_i).t;

        for plane_i = 1:plane_num
            B_sets_single = B_sets{lidar_pose_i, plane_i};
            n = plane.normal_vector_list{1, plane_i};
            d = plane.q_list{1, plane_i};

            T = eye(4, 4);
            T(1:3, 1:3) = R;
            T(1:3, 4) = t';
            B_sets_global = T * B_sets_single * T';

            total_error = total_error + [n;d]' * B_sets_global * [n;d];
            point_num = point_num + B_sets_single(4,4);
        end
    end
    
    total_error = total_error / point_num;

end