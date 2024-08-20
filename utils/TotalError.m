function total_error = TotalError(pose, point, plane, param)
    total_error = 0;
    for lidar_pose_i = 1:param.lidar_pose_num
        R = pose(lidar_pose_i).R;
        t = pose(lidar_pose_i).t;
        for plane_i = 1:param.plane_num
            point_cloud_single = point{lidar_pose_i, plane_i};
            n = plane.normal_vector_list{1, plane_i};
            d = plane.q_list{1, plane_i};

                e = (point_cloud_single * R' + t) * n + d;
                total_error = total_error + norm(e)^2;

        end
    end
    total_error = total_error / (param.lidar_pose_num * param.point_cloud_num);
end