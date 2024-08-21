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

function lidar_pose_estimate_vec = RecoverResultsFromCell(R_cell, t_cell, param)

    lidar_pose_estimate_cell = cell(1, param.lidar_pose_num);

    for lidar_pose_i = 1:param.lidar_pose_num
        axang = rotm2axang(R_cell{lidar_pose_i, 1});
        lidar_pose_estimate_cell{1, lidar_pose_i} = [axang(1:3) * axang(4), t_cell{lidar_pose_i, 1}'];
    end
    
    lidar_pose_estimate_vec = cell2mat(lidar_pose_estimate_cell);
    
end