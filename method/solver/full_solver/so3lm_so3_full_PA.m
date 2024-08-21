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

function [refined_pose_plane, Residual] = so3lm_so3_full_PA(lidar_pose_vec, plane_vec, B_sqrt_cell, param)

    initialized_pose_plane = [lidar_pose_vec, plane_vec];
    opt_func = @(x)ej_PA_full_so3_func(x, B_sqrt_cell);
    update_func = @(x, dx)so3_full_update_func(x, dx, param);
    [refined_pose_plane, Residual] = SO3LM(opt_func, update_func, initialized_pose_plane, 'SpecifyObjectiveGradient', true);

end