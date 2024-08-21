% This function is part of the GlobalPointer method as described in [1]. When you
% use this code, you are required to cite [1].
% 
% [1] GlobalPointer: Large-Scale Plane Adjustment with Bi-Convex Relaxation
% Author: B. Liao, Z. Zhao, L. Chen, H. Li, D. Cremers, P. Liu.
% European Conference on Computer Vision 2024 (ECCV 2024)
%
% 
% Author & Copyright (C) 2024: Bangyan Liao (liaobangyan[at]westlake[dot]edu[dot]cn)
%                              Zhenjun Zhao (ericzzj89[at]gmail[dot]com)
%                              Peidong Liu (liupeidong[at]westlake[dot]edu[dot]cn)

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