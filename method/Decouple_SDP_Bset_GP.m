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

function [solution_struct, time_record, error_record, Residual] = Decouple_SDP_Bset_GP(lidar_pose_struct, plane_struct, B_sets, runtime_param, param)

    % ----------- optimization preparation -----------
    [lidar_pose_error_struct, plane_error_struct] = PrepareVariable(lidar_pose_struct, plane_struct, param);
    [lidar_pose_error_vec, plane_error_vec] = struct2vec(lidar_pose_error_struct, plane_error_struct, param);
    
    % ----------- Redundant Constraint initialization -----------
    [RedundantA, RedundantB] = GetRotationRedundantConstraint();
    
    % ----------- Alternating Minimization -----------
    error_record.total_error_list = [];
    error_record.RotationError_list = [];
    error_record.TranslationError_list = [];
    error_record.nError_list = [];
    error_record.dError_list = [];
    error_record.optimal_pose_list = [];
    error_record.optimal_plane_list = [];
    error_record.pose_list = cell(20,1);

    tic;
    for iter = 1:runtime_param.max_iteration
        % ---- plane only sdp ----
        C_sets = GetCSet_parallel_mex_cell(B_sets, lidar_pose_error_vec);
        X_list = plane_sdp(C_sets, param);
        [plane_estimate_struct, plane_estimate_vec, optimal_plane_vec] = UpdatePlaneResults(X_list, param);
        
        % ---- pose only sdp ----
        D_sets = GetDSet_parallel_mex_cell(B_sets, plane_estimate_vec);
        Y_list = pose_sdp(D_sets, RedundantA, RedundantB, param);
        [lidar_pose_estimate_struct, lidar_pose_estimate_vec, optimal_pose_vec] = UpdatePoseResults(Y_list, param);

        % ---- recover results ----
        [lidar_pose_error_struct, plane_error_struct] = RecoverResults(lidar_pose_estimate_struct, plane_estimate_struct, param);
        [lidar_pose_error_vec, plane_error_vec] = struct2vec(lidar_pose_error_struct, plane_error_struct, param);
        error_record.pose_list{iter, 1} = lidar_pose_error_struct;
        
        % ---- calculate error ----
        total_error = TotalErrorBset(lidar_pose_error_struct, B_sets, plane_error_struct);
        [RotationError, TranslationError] = PoseError(lidar_pose_error_struct, lidar_pose_struct, param);
        [nError, dError] = PlaneError(plane_error_struct, plane_struct, param);
    
        % ---- results save ----
        error_record.total_error_list = [error_record.total_error_list, total_error];
        error_record.RotationError_list = [error_record.RotationError_list, RotationError];
        error_record.TranslationError_list = [error_record.TranslationError_list, TranslationError];
        error_record.nError_list = [error_record.nError_list, nError];
        error_record.dError_list = [error_record.dError_list, dError];
        error_record.optimal_pose_list = [error_record.optimal_pose_list; optimal_pose_vec];
        error_record.optimal_plane_list = [error_record.optimal_plane_list; optimal_plane_vec];

        % ---- stop check ----
        if StopCheck(error_record.total_error_list, runtime_param)
            break;
        end
    end

    solution_struct.plane_struct = plane_error_struct;
    solution_struct.pose_struct = lidar_pose_error_struct;
    time_record = toc;
    Residual = iter;

    % ---- results save ----
    error_record.total_error_list = error_record.total_error_list(1, end);
    error_record.RotationError_list = error_record.RotationError_list(1, end);
    error_record.TranslationError_list = error_record.TranslationError_list(1, end);
    error_record.nError_list = error_record.nError_list(1, end);
    error_record.dError_list = error_record.dError_list(1, end);

end
