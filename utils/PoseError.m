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

function [RotationError, TranslationError] = PoseError(pose, pose_gt, param)

    RotationError = 0;
    TranslationError = 0;

    for lidar_pose_i = 1:param.lidar_pose_num
        R = pose(lidar_pose_i).R;
        t = pose(lidar_pose_i).t;
        R_gt = pose_gt(lidar_pose_i).R_gt;
        t_gt = pose_gt(lidar_pose_i).t_gt;
        TranslationError = TranslationError + norm(t - t_gt)^2;
        r_matrix_error = rotm2axang(R' * R_gt);
        RotationError = RotationError + r_matrix_error(4)^2;
    end

    RotationError = RotationError / param.lidar_pose_num;
    TranslationError = TranslationError / param.lidar_pose_num;

end