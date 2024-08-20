function [F,J] = ej_plane_euc_func(x, y, point_cloud_cell)

    lidar_pose_num = size(point_cloud_cell, 1);
    plane_num = size(point_cloud_cell, 2);
    
    F_cell = cell(lidar_pose_num, plane_num);
    J_cell = cell(lidar_pose_num, plane_num);


    for lidar_pose_i = 1:lidar_pose_num
        pose_id = (lidar_pose_i - 1) * 6;
        rx = x(1,pose_id+1);
        ry = x(1,pose_id+2);
        rz = x(1,pose_id+3);
        tx = x(1,pose_id+4);
        ty = x(1,pose_id+5);
        tz = x(1,pose_id+6);
        
        for plane_i = 1:plane_num
            plane_id = (plane_i - 1) * 4;
            nx = y(1,plane_id+1);
            ny = y(1,plane_id+2);
            nz = y(1,plane_id+3);
            nq = y(1,plane_id+4);
            
            obs = point_cloud_cell{lidar_pose_i, plane_i};
            F_single = zeros(1, size(obs, 1));
            J_single = zeros(size(obs, 1), plane_num * 4);
            for obs_i = 1:size(obs, 1)
                P1 = obs(obs_i,1);
                P2 = obs(obs_i,2);
                P3 = obs(obs_i,3);
                    
                F_single(1, obs_i) = error_func(P1,P2,P3,nq,nx,ny,nz,rx,ry,rz,tx,ty,tz);
                J_plane = J_plane_func(P1,P2,P3,rx,ry,rz,tx,ty,tz);
                J_tmp = zeros(1, plane_num * 4);
                J_tmp(:,plane_id+1:plane_id+4) = J_plane;
                J_single(obs_i, :) = J_tmp;
            end
            F_cell{lidar_pose_i, plane_i} = F_single;
            J_cell{lidar_pose_i, plane_i} = J_single;
        end
    end


    F_cell = reshape(F_cell, [1, lidar_pose_num * plane_num]);
    J_cell = reshape(J_cell, [lidar_pose_num * plane_num, 1]);

    F_reg = zeros(1, plane_num);
    J_reg = zeros(plane_num, plane_num * 4);
    for plane_i = 1:plane_num
        plane_id = (plane_i - 1) * 4;
        nx = y(1,plane_id+1);
        ny = y(1,plane_id+2);
        nz = y(1,plane_id+3);
        nq = y(1,plane_id+4);
        F_reg(1, plane_i) = reg_error_func(nx,ny,nz);
        J_single_tmp = reg_J_plane_func(nx,ny,nz);
        J_tmp = zeros(1, plane_num * 4);
        J_tmp(:,plane_id+1:plane_id+3) = J_single_tmp;
        J_reg(plane_i, :) = J_tmp;
    end


    F = [cell2mat(F_cell),F_reg];
    J = [cell2mat(J_cell);J_reg];

    
end