function [F,J] = ej_PA_plane_so3_func(x, pose, B_sqrt_cell)
    
    
    lidar_pose_num = size(B_sqrt_cell, 1);
    plane_num = 1;
    
    F_cell = cell(lidar_pose_num, plane_num);
    J_cell = cell(lidar_pose_num, plane_num);



    nq = x(3);
    phi = x(1);
    theta = x(2);




    for lidar_pose_i = 1:lidar_pose_num
        pose_id = (lidar_pose_i - 1) * 6;
        r_vec = pose(1,pose_id+1:pose_id+3);
        R = axang2rotm([r_vec / norm(r_vec) norm(r_vec)]);
        t = pose(1,pose_id+4:pose_id+6);


        B_sqrt = B_sqrt_cell{lidar_pose_i, 1};



        T = [R,t';0 0 0 1];

        n = [sin(phi)*cos(theta);sin(phi)*sin(theta);cos(phi);nq];

        % ----------------- error definition -----------------
        F_single = n' * T * B_sqrt;


        J_tmp_n = (T * B_sqrt)';
        J_tmp_n_phi = cos(theta)*cos(phi)*J_tmp_n(:,1) + sin(theta)*cos(phi)*J_tmp_n(:,2) - sin(phi)*J_tmp_n(:,3);
        J_tmp_n_theta = -sin(theta)*sin(phi)*J_tmp_n(:,1) + sin(phi)*cos(theta)*J_tmp_n(:,2);
        J_tmp_n_nq = J_tmp_n(:,4);




        F_cell{lidar_pose_i, 1} = F_single;
        J_cell{lidar_pose_i, 1} = [J_tmp_n_phi,J_tmp_n_theta,J_tmp_n_nq];
    end



    F_cell = reshape(F_cell, [1, lidar_pose_num * plane_num]);
    J_cell = reshape(J_cell, [lidar_pose_num * plane_num, 1]);


    F = cell2mat(F_cell);
    J = cell2mat(J_cell);
end

