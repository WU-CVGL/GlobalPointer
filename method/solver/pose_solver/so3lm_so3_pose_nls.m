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

function refined_pose_plane = so3lm_so3_pose_nls(lidar_pose_vec, plane_vec, point_cloud_cell, param)

    opt_func = @(x)ej_pose_so3_func(x, plane_vec, point_cloud_cell);
    update_func = @(x, dx)so3_pose_update_func(x, dx, param);
    [refined_pose_plane] = SO3LM(opt_func, update_func, lidar_pose_vec, 'SpecifyObjectiveGradient', true);
    
end