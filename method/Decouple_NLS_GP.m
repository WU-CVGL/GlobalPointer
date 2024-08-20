function [solution_struct, time_record, error_record, Residual] = Decouple_NLS_GP(lidar_pose_struct, plane_struct, point_cloud_cell, runtime_param, param)
    
    [lidar_pose_estimate_vec, plane_estimate_vec] = PrepareVariableVec(lidar_pose_struct, plane_struct, param);

    error_record.total_error_list = [];
    error_record.RotationError_list = [];
    error_record.TranslationError_list = [];
    error_record.nError_list = [];
    error_record.dError_list = [];
    tic
    

    for iter = 1:runtime_param.max_iteration
        
        
        % ------------- pose solver -------------
        % ----- so3lm_euc_pose_nls -----
        lidar_pose_estimate_vec = so3lm_so3_pose_nls(lidar_pose_estimate_vec, plane_estimate_vec, point_cloud_cell, param);

        % ------------- plane solver -------------
        % ----- so3lm_plane_nls -----
        plane_estimate_vec = so3lm_plane_nls(lidar_pose_estimate_vec, plane_estimate_vec, point_cloud_cell);
        


        % ---- update results ----
        [lidar_pose_estimate_struct, plane_estimate_struct] = UpdateResults([lidar_pose_estimate_vec, plane_estimate_vec], lidar_pose_struct, plane_struct, param);
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