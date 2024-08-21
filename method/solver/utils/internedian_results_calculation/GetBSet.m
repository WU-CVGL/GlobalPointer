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

function B_sets = GetBSet(point_cloud_cell, param)

    B_sets = cell(param.lidar_pose_num, param.plane_num);

    for plane_i = 1:param.plane_num
        for lidar_pose_i = 1:param.lidar_pose_num
            point_single = point_cloud_cell{lidar_pose_i, plane_i};
            point_single_homo = [point_single, ones(size(point_single,1),1)];
            B_sets{lidar_pose_i, plane_i} = point_single_homo' * point_single_homo;
        end
    end

end