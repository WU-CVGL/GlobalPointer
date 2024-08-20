function [solution_struct, time_record, error_record, Residual] = Full_PA_GP(lidar_pose_struct, plane_struct, B_sets, runtime_param, param)
    
    [lidar_pose_estimate_vec, plane_estimate_vec] = PrepareVariableVec(lidar_pose_struct, plane_struct, param);

    plane_estimate_vec = Euc2Sphere(plane_estimate_vec, param);

    error_record.total_error_list = [];
    error_record.RotationError_list = [];
    error_record.TranslationError_list = [];
    error_record.nError_list = [];
    error_record.dError_list = [];
    tic


    B_sqrt_cell = cellfun(@(B)(chol(B)'), B_sets, 'UniformOutput',false);


    % ------------- so3lm_so3_full_nls -------------
    [refined_pose_plane_so3lm, Residual] = so3lm_so3_full_PA(lidar_pose_estimate_vec, plane_estimate_vec, B_sqrt_cell, param);

    refined_pose_plane_so3lm_plane = Sphere2Euc(refined_pose_plane_so3lm(1, end-3*param.plane_num+1:end), param);
    refined_pose_plane_so3lm = [refined_pose_plane_so3lm(1, 1:6*param.lidar_pose_num), refined_pose_plane_so3lm_plane];


    % ---- update results ----
    [lidar_pose_estimate_struct, plane_estimate_struct] = UpdateResults(refined_pose_plane_so3lm, lidar_pose_struct, plane_struct, param);

    % ---- recover results ----
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




    solution_struct = [];
    time_record = toc;


end