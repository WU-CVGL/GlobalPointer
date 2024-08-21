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

function [refined_pose] = so3lm_so3_pose_PA(lidar_pose_vec, plane_vec, B_sqrt_cell, param)

    initialized_pose = reshape(lidar_pose_vec, [6, param.lidar_pose_num])';
    initialized_pose = mat2cell(initialized_pose, ones(1,param.lidar_pose_num));
    refined_pose = cell(param.lidar_pose_num, 1);

    lidar_pose_num = param.lidar_pose_num;
    param.lidar_pose_num = 1;

    for i = 1:lidar_pose_num
        opt_func = @(x)ej_PA_pose_so3_func(x, plane_vec, B_sqrt_cell{i,1});
        update_func = @(x, dx)so3_pose_update_func(x, dx, param);
        [refined_pose{i,1}] = SO3LM(opt_func, update_func, initialized_pose{i,1}, 'SpecifyObjectiveGradient', true);
    end

    param.lidar_pose_num = lidar_pose_num;
    refined_pose = cell2mat(refined_pose);
    refined_pose = reshape(refined_pose', [6 * param.lidar_pose_num, 1])';

end