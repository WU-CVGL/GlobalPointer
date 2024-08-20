function [plane_struct, lidar_pose_struct] = pose_plane_rand(param, plane_struct, lidar_pose_struct)


    % -------- pose data generate --------    
    for lidar_pose_i = 1:param.lidar_pose_num

        if lidar_pose_i == 1
            lidar_pose_struct(lidar_pose_i).R = eye(3) ;
            lidar_pose_struct(lidar_pose_i).t = zeros(1, 3);
        else
            noise_r = randn(1, 3);
            lidar_pose_struct(lidar_pose_i).R = lidar_pose_struct(lidar_pose_i).R * axang2rotm([noise_r / norm(noise_r), 0.5 * norm(noise_r)]);
            lidar_pose_struct(lidar_pose_i).t = lidar_pose_struct(lidar_pose_i).t + 2 * randn(1, 3);
        end
        
        


    end



    for plane_i = 1:param.plane_num

        plane_struct.normal_vector_list{1, plane_i} = randn(3, 1);
        plane_struct.q_list{1, plane_i} = randn(3, 1);


    end


   
end