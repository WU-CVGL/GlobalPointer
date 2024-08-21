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

function lidar_pose_vec = Tcell2Tvec(lidar_pose_cell, param)

    lidar_pose_vec = [];
    
    for lidar_pose_i = 1:param.lidar_pose_num
        T = lidar_pose_cell{1, lidar_pose_i};
        r = rotm2axang(T(1:3, 1:3));
        t = T(1:3, 4);
        lidar_pose_vec = [lidar_pose_vec, r(1, 1:3) * r(1, 4), t'];
    end

end