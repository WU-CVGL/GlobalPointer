function [lidar_pose_estimate_struct, lidar_pose_estimate_matrix, optimal_pose_vec] = UpdatePoseResults(Y_list, param)

    optimal_pose_vec = ones(1, param.lidar_pose_num);
    for lidar_pose_i = 1:param.lidar_pose_num
        [U,S,V] = svd(Y_list{1, lidar_pose_i});
        % [d,ind] = sort(diag(S));
        % Ss = S(ind,ind);
        % Vs = V(:,ind);
        % A = sqrt(Ss)*Vs';
        % x_solution = A(4, :);
        % x_solution = x_solution / x_solution(1, 13);

        A = S*V';
        x_solution = A(1, :);
        x_solution = x_solution / x_solution(1, 13);

        R = [x_solution(1, 1:3); x_solution(1, 4:6); x_solution(1, 7:9)];
        t = x_solution(1, 10:12);
        R_near = SearchNearestRotation(R);
        lidar_pose_estimate_struct(lidar_pose_i).R = R_near;
        lidar_pose_estimate_struct(lidar_pose_i).t = t;


        % if Ss(1,1) / Ss(2,2) < 9
        %     optimal_pose_vec(1, lidar_pose_i) = 0;
        % end
    end


    lidar_pose_estimate_matrix = cell(param.lidar_pose_num,1);
    for lidar_pose_i = 1:param.lidar_pose_num
        Tmp = zeros(4,4);
        Tmp(1:3,1:3) = lidar_pose_estimate_struct(lidar_pose_i).R;
        Tmp(1:3,4) = lidar_pose_estimate_struct(lidar_pose_i).t';
        Tmp(4,4) = 1;
        lidar_pose_estimate_matrix{lidar_pose_i} = Tmp;
    end
end