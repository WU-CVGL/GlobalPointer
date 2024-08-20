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