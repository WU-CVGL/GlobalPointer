function [F,J] = ej_PA_full_so3_func(x, B_sqrt_cell)
    
    
    lidar_pose_num = size(B_sqrt_cell, 1);
    plane_num = size(B_sqrt_cell, 2);
    
    F_cell = cell(lidar_pose_num, plane_num);
    J_cell = cell(lidar_pose_num, plane_num);


    for lidar_pose_i = 1:lidar_pose_num
        pose_id = (lidar_pose_i - 1) * 6;
        r_vec = x(1,pose_id+1:pose_id+3);
        R = axang2rotm([r_vec / norm(r_vec) norm(r_vec)]);
        t = x(1,pose_id+4:pose_id+6);

        rx = r_vec(1);
        ry = r_vec(2);
        rz = r_vec(3);
        tx = t(1);
        ty = t(2);
        tz = t(3);
        
        for plane_i = 1:plane_num
            plane_id = lidar_pose_num * 6 + (plane_i - 1) * 3;
            n_spher = x(1,plane_id+1:plane_id+2);
            nq = x(1,plane_id+3);
            phi = n_spher(1);
            theta = n_spher(2);

            B_sqrt = B_sqrt_cell{lidar_pose_i, plane_i};
            
            F_single = zeros(1, 4);
            J_single = zeros(4, plane_num * 3 + lidar_pose_num * 6);

            B_sqrt11 = B_sqrt(1,1);
            B_sqrt12 = B_sqrt(1,2);
            B_sqrt13 = B_sqrt(1,3);
            B_sqrt14 = B_sqrt(1,4);
            B_sqrt21 = B_sqrt(2,1);
            B_sqrt22 = B_sqrt(2,2);
            B_sqrt23 = B_sqrt(2,3);
            B_sqrt24 = B_sqrt(2,4);
            B_sqrt31 = B_sqrt(3,1);
            B_sqrt32 = B_sqrt(3,2);
            B_sqrt33 = B_sqrt(3,3);
            B_sqrt34 = B_sqrt(3,4);
            B_sqrt41 = B_sqrt(4,1);
            B_sqrt42 = B_sqrt(4,2);
            B_sqrt43 = B_sqrt(4,3);
            B_sqrt44 = B_sqrt(4,4);


            T = [R,t';0 0 0 1];

            n = [sin(phi)*cos(theta);sin(phi)*sin(theta);cos(phi);nq];
            
            % ----------------- error definition -----------------
            F_single = n' * T * B_sqrt;


            F_single = PA_error_func(B_sqrt11,B_sqrt12,B_sqrt13,B_sqrt14,B_sqrt21,B_sqrt22,B_sqrt23,B_sqrt24,B_sqrt31,B_sqrt32,B_sqrt33,B_sqrt34,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44,nq,phi,rx,ry,rz,theta,tx,ty,tz);
            
            J_tmp = J_PA_full_func(B_sqrt11,B_sqrt12,B_sqrt13,B_sqrt14,B_sqrt21,B_sqrt22,B_sqrt23,B_sqrt24,B_sqrt31,B_sqrt32,B_sqrt33,B_sqrt34,B_sqrt41,B_sqrt42,B_sqrt43,B_sqrt44,phi,rx,ry,rz,theta,tx,ty,tz);

%             B_sqrt_T = T * B_sqrt;
%             J_tmp_r = (-[0 -n(3) n(2); n(3) 0 -n(1); -n(2) n(1) 0] * B_sqrt_T(1:3,:))';
%             J_tmp_t = (n(1:3,1) * B_sqrt(4,:))';
%             J_tmp_n = (T * B_sqrt)';
%             J_tmp_n_phi = cos(theta)*cos(phi)*J_tmp_n(:,1) + sin(theta)*cos(phi)*J_tmp_n(:,2) - sin(phi)*J_tmp_n(:,3);
%             J_tmp_n_theta = -sin(theta)*sin(phi)*J_tmp_n(:,1) + sin(phi)*cos(theta)*J_tmp_n(:,2);
%             J_tmp_n_nq = J_tmp_n(:,4);

% 
%             J_single(:,pose_id+1:pose_id+3) = J_tmp_r;
%             J_single(:,pose_id+4:pose_id+6) = J_tmp_t;
%             J_single(:,plane_id+1:plane_id+3) = [J_tmp_n_phi,J_tmp_n_theta,J_tmp_n_nq];

            J_single(:,pose_id+1:pose_id+3) = J_tmp(:,1:3);
            J_single(:,pose_id+4:pose_id+6) = J_tmp(:,4:6);
            J_single(:,plane_id+1:plane_id+3) = J_tmp(:,7:9);


            F_cell{lidar_pose_i, plane_i} = F_single;
            J_cell{lidar_pose_i, plane_i} = J_single;
        end
    end


    F_cell = reshape(F_cell, [1, lidar_pose_num * plane_num]);
    J_cell = reshape(J_cell, [lidar_pose_num * plane_num, 1]);


    F = cell2mat(F_cell);
    J = cell2mat(J_cell);
end


function S = skew_symmetric(s)
    S = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0];
end