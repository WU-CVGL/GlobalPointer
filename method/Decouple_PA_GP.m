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

function [solution_struct, time_record, error_record, Residual] = Decouple_PA_GP(lidar_pose_struct, plane_struct, B_sets, runtime_param, param)
    
    [lidar_pose_estimate_vec, plane_estimate_vec] = PrepareVariableVec(lidar_pose_struct, plane_struct, param);

    error_record.total_error_list = [];
    error_record.RotationError_list = [];
    error_record.TranslationError_list = [];
    error_record.nError_list = [];
    error_record.dError_list = [];
    
    tic;
    B_sqrt_cell = B_sets;
    for i = 1:size(B_sets, 1)
        for j = 1:size(B_sets, 2)
            if sum(B_sets{i, j},'all') == 0
                B_sqrt_cell{i, j} = zeros(4, 4);
            else
                B_sqrt_cell{i, j} = chol(B_sets{i,j}+diag([1, 1, 1, 1]) * 1e-5)';
            end
            
        end
    end

    B_sqrt_pose_cell = cell(param.lidar_pose_num, 1);
    B_sqrt_plane_cell = cell(1, param.plane_num);

    for i = 1:param.lidar_pose_num
        B_sqrt_pose_cell{i, 1} = B_sqrt_cell(i,:);
    end

    for i = 1:param.plane_num
        B_sqrt_plane_cell{1, i} = B_sqrt_cell(:,i);
    end

    for iter = 1:200
        plane_estimate_vec = Euc2Sphere(plane_estimate_vec, param);
        
        % ------------- pose solver -------------
        lidar_pose_estimate_vec = so3lm_so3_pose_PA(lidar_pose_estimate_vec, plane_estimate_vec, B_sqrt_pose_cell, param);

        % ------------- plane solver -------------
        plane_estimate_vec = so3lm_so3_plane_PA(lidar_pose_estimate_vec, plane_estimate_vec, B_sqrt_plane_cell, param);
        plane_estimate_vec = Sphere2Euc(plane_estimate_vec, param);
        
        % ---- update results ----
        [lidar_pose_estimate_struct, plane_estimate_struct] = UpdateResults([lidar_pose_estimate_vec, plane_estimate_vec], lidar_pose_struct, plane_struct, param);
        
        % ---- recover results ----
        [lidar_pose_estimate_vec, plane_estimate_vec] = PrepareVariableVec(lidar_pose_estimate_struct, plane_estimate_struct, param);
        [lidar_pose_estimate_struct, plane_estimate_struct] = RecoverResults(lidar_pose_estimate_struct, plane_estimate_struct, param);
        
        % ---- calculate error ----
        total_error = TotalErrorBset(lidar_pose_estimate_struct, B_sets, plane_estimate_struct, param);
        [RotationError, TranslationError] = PoseError(lidar_pose_estimate_struct, lidar_pose_struct, param);
        [nError, dError] = PlaneError(plane_estimate_struct, plane_struct, param);

        % ---- results save ----
        error_record.total_error_list = [error_record.total_error_list, total_error];
        error_record.RotationError_list = [error_record.RotationError_list, RotationError];
        error_record.TranslationError_list = [error_record.TranslationError_list, TranslationError];
        error_record.nError_list = [error_record.nError_list, nError];
        error_record.dError_list = [error_record.dError_list, dError];

        % ---- stop check ----
        if StopCheck(error_record.total_error_list, runtime_param)
            break;
        end
    end

    Residual = error_record.total_error_list; 

    % ---- results save ----
    error_record.total_error_list = error_record.total_error_list(1, end);
    error_record.RotationError_list = error_record.RotationError_list(1, end);
    error_record.TranslationError_list = error_record.TranslationError_list(1, end);
    error_record.nError_list = error_record.nError_list(1, end);
    error_record.dError_list = error_record.dError_list(1, end);
    solution_struct.plane_struct = plane_estimate_struct;
    solution_struct.pose_struct = lidar_pose_estimate_struct;
    time_record = toc;

end