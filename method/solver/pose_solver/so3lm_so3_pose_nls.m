function refined_pose_plane = so3lm_so3_pose_nls(lidar_pose_vec, plane_vec, point_cloud_cell, param)

    opt_func = @(x)ej_pose_so3_func(x, plane_vec, point_cloud_cell);
    update_func = @(x, dx)so3_pose_update_func(x, dx, param);
    [refined_pose_plane] = SO3LM(opt_func, update_func, lidar_pose_vec, 'SpecifyObjectiveGradient', true);


end