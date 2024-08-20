function [refined_plane] = so3lm_so3_plane_PA(lidar_pose_vec, plane_vec, B_sqrt_cell, param)

    initialized_plane = reshape(plane_vec, [3, param.plane_num])';

    initialized_plane = mat2cell(initialized_plane, ones(1,param.plane_num));

    refined_plane = cell(param.plane_num, 1);


    for i = 1:param.plane_num
        opt_func = @(x)ej_PA_plane_so3_func(x, lidar_pose_vec, B_sqrt_cell{1,i});
        update_func = @(x, dx)euc_update_func(x, dx);
        [refined_plane{i,1}] = SO3LM(opt_func, update_func, initialized_plane{i,1}, 'SpecifyObjectiveGradient', true);
    end

    refined_plane = cell2mat(refined_plane);
    refined_plane = reshape(refined_plane', [3 * param.plane_num, 1])';
end