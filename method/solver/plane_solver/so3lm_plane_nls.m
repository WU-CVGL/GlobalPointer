function refined_pose_plane = so3lm_plane_nls(lidar_pose_vec, plane_vec, point_cloud_cell)

    opt_func = @(x)ej_plane_euc_func(lidar_pose_vec, x, point_cloud_cell);
    update_func = @(x, dx)euc_update_func(x, dx);
    [refined_pose_plane] = SO3LM(opt_func, update_func, plane_vec, 'SpecifyObjectiveGradient', true);
end