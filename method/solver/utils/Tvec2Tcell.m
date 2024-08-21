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

function lidar_pose_cell = Tvec2Tcell(lidar_pose_vec, param)

    lidar_pose_cell = cell(1, param.lidar_pose_num);

    for lidar_pose_i = 1:param.lidar_pose_num
        pose_id = (lidar_pose_i - 1) * 6;
        r = lidar_pose_vec(pose_id + (1:3));
        t = lidar_pose_vec(pose_id + (4:6));

        if norm(r) == 0
            R = eye(3);
        else
            R = axang2rotm([r / norm(r), norm(r)]);
        end
        
        T = [R, t'; 0 0 0 1];
        lidar_pose_cell{1, lidar_pose_i} = T;
    end

end