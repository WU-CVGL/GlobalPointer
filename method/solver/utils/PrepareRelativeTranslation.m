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

function lidar_relative_translation_cell = PrepareRelativeTranslation(lidar_pose_struct, param)

    lidar_relative_translation_cell = cell(param.lidar_pose_num, param.lidar_pose_num);

    for lidar_pose_i = 1:param.lidar_pose_num
        for lidar_pose_j = 1:param.lidar_pose_num
            lidar_relative_translation_cell{lidar_pose_i, lidar_pose_j} = lidar_pose_struct(lidar_pose_i).R' ...
                * (lidar_pose_struct(lidar_pose_j).t' - lidar_pose_struct(lidar_pose_i).t');
        end
    end

end