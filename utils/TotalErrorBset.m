function total_error = TotalErrorBset(pose, B_sets, plane, param)
    total_error = 0;
    point_num = 0;

    lidar_pose_num = size(B_sets, 1);
    plane_num = size(B_sets, 2);
    
    for lidar_pose_i = 1:lidar_pose_num
        R = pose(lidar_pose_i).R;
        t = pose(lidar_pose_i).t;
        for plane_i = 1:plane_num
            B_sets_single = B_sets{lidar_pose_i, plane_i};
            n = plane.normal_vector_list{1, plane_i};
            d = plane.q_list{1, plane_i};

            T = eye(4,4);
            T(1:3,1:3) = R;
            T(1:3,4) = t';
            B_sets_global = T * B_sets_single * T';

            total_error = total_error + [n;d]' * B_sets_global * [n;d];
            point_num = point_num + B_sets_single(4,4);
        end
    end
    total_error = total_error / point_num;
    % tic
    % B_counts = cell(size(B_sets));
    % parfor i = 1:numel(B_counts)
    %     B_counts{i} = B_sets{i}(4,4);
    % end
    % 
    % B_sets_global = cell(size(B_sets));
    % parfor lidar_i = 1:param.lidar_pose_num
    %     B_sets_global(lidar_i, :) = cellfun(@(B)([pose(lidar_i).R,pose(lidar_i).t';0 0 0 1] * B * [pose(lidar_i).R',zeros(3,1);pose(lidar_i).t,1]), ...
    %         B_sets(lidar_i, :), 'UniformOutput',false);
    % end
    % 
    % 
    % error_local = cell(size(B_sets));
    % parfor plane_i = 1:param.plane_num
    %     error_local(plane_i, :) = cellfun(@(B)([plane.normal_vector_list{1, plane_i};plane.q_list{1, plane_i}]' * B * [plane.normal_vector_list{1, plane_i};plane.q_list{1, plane_i}]), ...
    %         B_sets_global(:, plane_i), 'UniformOutput',false);
    % end
    % 
    % error_local = cell2mat(error_local);
    % B_counts = cell2mat(B_counts);
    % total_error = sum(error_local,"all") / sum(B_counts, "all");
    % toc
end