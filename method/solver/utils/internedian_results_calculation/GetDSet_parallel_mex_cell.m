function D_sets = GetDSet_parallel_mex_cell(B_sets, plane_estimate_struct)


    lidar_pose_num = size(B_sets, 1);
    plane_num = size(B_sets, 2);

    A_sets = cell(1, plane_num);

    for plane_i = 1:plane_num
        A_sets{1, plane_i} = plane_estimate_struct(:,plane_i) * plane_estimate_struct(:,plane_i)';
    end
    


    D_sets = cell(1, lidar_pose_num);
    for lidar_pose_i = 1:lidar_pose_num
        D = zeros(16, 16);
        for plane_i = 1:plane_num
            D = D + kron(A_sets{1, plane_i},B_sets{lidar_pose_i, plane_i});
        end
        D_sets{1, lidar_pose_i} = D;
    end


    % tic
    % D_sets = cell(1, param.lidar_pose_num);
    % parfor lidar_pose_i = 1:param.lidar_pose_num
    %     D = zeros(16, 16);
    %     for plane_i = 1:param.plane_num
    %         D = D + kron(A_sets{1, plane_i},B_sets{lidar_pose_i, plane_i});
    %     end
    %     D_sets{1, lidar_pose_i} = D;
    % end
    % toc

end