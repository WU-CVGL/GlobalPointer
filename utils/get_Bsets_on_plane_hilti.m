function [plane_struct, lidar_pose_struct, B_sets, param] = get_Bsets_on_plane_hilti(param, seq_name)

    dataset_rootpath = param.dataset_rootpath + seq_name + "\";
    pcd_root_path = dataset_rootpath + "benchmark_realworld\";
    pose_file = pcd_root_path + "alidarPose.csv";
    plane_path = dataset_rootpath + "planes_id\";
    Bsets_path = dataset_rootpath + "Bsets.mat";


    % ----- pose load -----
    pose_list_raw = load(pose_file);
    pose_num = size(pose_list_raw, 1) / 4;
    pose_list = mat2cell(pose_list_raw,repmat(4,[pose_num,1]));
    time_stamp_func = @(x) x(4,4);
    time_stamp_list = cellfun(time_stamp_func,pose_list);
    


    % -------- pose data generate --------
    lidar_pose_struct = [];
    for lidar_pose_i = 1:pose_num
        pose_single = pose_list{lidar_pose_i, 1};
        lidar_pose_struct(lidar_pose_i).R_gt = pose_single(1:3,1:3);
        lidar_pose_struct(lidar_pose_i).t_gt = pose_single(1:3,4)';
    end


    % -------- loda B sets --------
    load(Bsets_path);

    param.plane_num = size(B_sets,2);
    param.lidar_pose_num = size(B_sets,1);


    if param.fix_lidar_scan ~= 0 && param.lidar_pose_num >= param.fix_lidar_scan
        overlap_winsize = 4 * ceil(param.lidar_pose_num / param.fix_lidar_scan) + 1;
        %overlap_winsize = 90;
        win_id_cell = cell(param.fix_lidar_scan, 1);
        fix_id_cell = zeros(1, param.fix_lidar_scan);
        for i = 1:param.fix_lidar_scan
            fix_id_cell(1,i) = min(ceil(param.lidar_pose_num / param.fix_lidar_scan * i), param.lidar_pose_num);
            first_id = max(1, fix_id_cell(1,i) - (overlap_winsize-1) / 2);
            sec_id = min(param.lidar_pose_num, fix_id_cell(1,i) + (overlap_winsize-1) / 2);
            %win_id_cell{i, 1} = linspace(first_id,sec_id,sec_id-first_id+1);
            win_id_cell{i, 1} = randperm(param.lidar_pose_num,overlap_winsize);
        end

        B_sets_new = cell(param.fix_lidar_scan, param.plane_num);
        for i = 1:param.fix_lidar_scan
            fix_id = fix_id_cell(1,i);

            T_tar = eye(4);
            T_tar(1:3,1:3) = lidar_pose_struct(fix_id).R_gt;
            T_tar(1:3,4) = lidar_pose_struct(fix_id).t_gt;

            win_id_list = win_id_cell{i, 1};
            for j = 1:size(win_id_list, 2)
                win_id = win_id_list(1, j);
                T_src = eye(4);
                T_src(1:3,1:3) = lidar_pose_struct(win_id).R_gt;
                T_src(1:3,4) = lidar_pose_struct(win_id).t_gt;
                T = inv(T_tar) * T_src;
                for plane_i = 1:param.plane_num
                    if isempty(B_sets_new{i, plane_i})
                        B_sets_new{i, plane_i} = zeros(4,4);
                    end
                    B_sets_new{i, plane_i} = B_sets_new{i, plane_i} + T * B_sets{win_id,plane_i} * T';
                end
            end
        end
        

        param.lidar_pose_num = param.fix_lidar_scan;
        lidar_pose_struct = lidar_pose_struct(fix_id_cell);
        B_sets = B_sets_new;
    end




    B_m = zeros(param.lidar_pose_num, param.plane_num);
    for j = 1:param.lidar_pose_num
        for i = 1:param.plane_num
            B_single = B_sets{j, i};
            if B_single(4,4) ~= 0
                B_m(j, i) = 1;
            end
        end
    end

    



    % -------- plane struct data generate --------
    plane_struct.normal_vector_gt_list = cell(1, param.plane_num);
    plane_struct.q_gt_list = cell(1, param.plane_num);
    for plane_i = 1:param.plane_num
        B_single = zeros(4,4);
        for pose_i = 1:param.lidar_pose_num
            B = B_sets{pose_i, plane_i};
            if sum(B,"all") == 0
                continue
            end
            B_sets{pose_i, plane_i} = B / B(4,4);
            T = eye(4);
            T(1:3,1:3) = lidar_pose_struct(pose_i).R_gt;
            T(1:3,4) = lidar_pose_struct(pose_i).t_gt;
            B_single = B_single + T * B_sets{pose_i, plane_i} * T';
        end
        B_single = B_single / B_single(4,4);
        [V,D,W] = eig(B_single);

        [d,ind] = sort(diag(D));
        Ds = D(ind,ind);
        Vs = V(:,ind);
        nq = Vs(:,1);
        nq = nq / norm(nq(1:3));
        plane_struct.normal_vector_gt_list{1, plane_i} = nq(1:3);
        d = - nq(1:3)' * B_single(1:3,4);
        plane_struct.q_gt_list{1, plane_i} = B_single(1:3,4) / d * nq(4);
    end


    Dis_m = zeros(param.lidar_pose_num, param.lidar_pose_num);
    for j = 1:param.lidar_pose_num
        B_single = B_m(j, :);
        B_dis = B_m - B_single;
        Dis_single = vecnorm(B_dis');
        Dis_m(j,:) = Dis_single;
    end

    
    % if seq_name == "Basement4"
    % 
    %     rand_id = 759:param.lidar_pose_num;
    %     rand_id = randperm(param.lidar_pose_num - 759 + 1, param.lidar_pose_num - 759 + 1);
    %     rand_id = 759 + sort(rand_id) - 1;
    %     param.lidar_pose_num = param.lidar_pose_num - 759 + 1;
    %
    %     rand_id_num = vecnorm(B_m(rand_id,:));
    %     inlier_id = find(rand_id_num ~= 0);
    %     param.plane_num = size(inlier_id, 2);
    % 
    %     plane_struct.normal_vector_gt_list = plane_struct.normal_vector_gt_list(1,inlier_id);
    %     plane_struct.q_gt_list = plane_struct.q_gt_list(1,inlier_id);
    %     B_sets = B_sets(rand_id, inlier_id);
    %     lidar_pose_struct = lidar_pose_struct(1, rand_id);
    % end

    
    % if param.randn_lidar_scan ~= 0
    % 
    %     rand_id = randperm(param.lidar_pose_num,param.randn_lidar_scan);
    %     rand_id = sort(rand_id);
    %     rand_id = 401:450;
    % 
    % 
    %     rand_id_num = vecnorm(B_m(rand_id,:));
    %     inlier_id = find(rand_id_num ~= 0);
    % 
    % 
    %     param.lidar_pose_num = param.randn_lidar_scan;
    %     param.plane_num = size(inlier_id, 2);
    % 
    %     plane_struct.normal_vector_gt_list = plane_struct.normal_vector_gt_list(1,inlier_id);
    %     plane_struct.q_gt_list = plane_struct.q_gt_list(1,inlier_id);
    %     B_sets = B_sets(rand_id, inlier_id);
    %     lidar_pose_struct = lidar_pose_struct(1, rand_id);
    % end






   
end