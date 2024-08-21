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

function [solution_struct, time_record, error_record, Residual] = Full_NLS_GP(lidar_pose_struct, plane_struct, point_cloud_cell, runtime_param, param)
    
    [lidar_pose_estimate_vec, plane_estimate_vec] = PrepareVariableVec(lidar_pose_struct, plane_struct, param);

    error_record.total_error_list = [];
    error_record.RotationError_list = [];
    error_record.TranslationError_list = [];
    error_record.nError_list = [];
    error_record.dError_list = [];

    tic;
    % ------------- so3lm_so3_full_nls -------------
    [refined_pose_plane_so3lm, Residual] = so3lm_so3_full_nls(lidar_pose_estimate_vec, plane_estimate_vec, point_cloud_cell, param);
    Residual = Residual / (param.lidar_pose_num * param.point_cloud_num);

    % ---- update results ----
    [lidar_pose_estimate_struct, plane_estimate_struct] = UpdateResults(refined_pose_plane_so3lm, lidar_pose_struct, plane_struct, param);

    % ---- recover results ----
    [lidar_pose_estimate_struct, plane_estimate_struct] = RecoverResults(lidar_pose_estimate_struct, plane_estimate_struct, param);

    % ---- calculate error ----
    total_error = TotalError(lidar_pose_estimate_struct, point_cloud_cell, plane_estimate_struct, param);
    [RotationError, TranslationError] = PoseError(lidar_pose_estimate_struct, lidar_pose_struct, param);
    [nError, dError] = PlaneError(plane_estimate_struct, plane_struct, param);

    % ---- results save ----
    error_record.total_error_list = [error_record.total_error_list, total_error];
    error_record.RotationError_list = [error_record.RotationError_list, RotationError];
    error_record.TranslationError_list = [error_record.TranslationError_list, TranslationError];
    error_record.nError_list = [error_record.nError_list, nError];
    error_record.dError_list = [error_record.dError_list, dError];
    solution_struct = [];
    time_record = toc;

end