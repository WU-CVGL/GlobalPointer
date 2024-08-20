function [B_sets, inlier_laplace_matrix] = Outlier_Bset(B_sets, outlier_ratio, param)

    inlier_laplace_matrix = ones(size(B_sets,[1,2]));
    for lidar_pose_i = 1:param.lidar_pose_num
        outlier_rand_id = randperm(param.plane_num,param.plane_num*outlier_ratio);

        if ~isempty(outlier_rand_id)
            for i = 1:size(outlier_rand_id, 2)
                rand_1 = randn(1,4);
                rand_2 = randn(1,4);
                rand_3 = randn(1,4);
                rand_4 = randn(1,4);
                rand_m = rand_1' * rand_1 + rand_2' * rand_2 + rand_3' * rand_3 + rand_4' * rand_4;
                B_sets{lidar_pose_i, outlier_rand_id(1, i)} = rand_m * norm(B_sets{lidar_pose_i, outlier_rand_id(1, i)});
                inlier_laplace_matrix(lidar_pose_i, outlier_rand_id(1, i)) = 0;
            end
        end
    end
    



   
end