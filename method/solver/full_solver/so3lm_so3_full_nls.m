function [refined_pose_plane, Residual] = so3lm_so3_full_nls(lidar_pose_vec, plane_vec, point_cloud_cell, param)

    initialized_pose_plane = [lidar_pose_vec, plane_vec];
    opt_func = @(x)ej_full_so3_func(x, point_cloud_cell);
    update_func = @(x, dx)so3_full_update_func(x, dx, param);
    [refined_pose_plane, Residual] = SO3LM(opt_func, update_func, initialized_pose_plane, 'SpecifyObjectiveGradient', true);


end