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

function [nError, dError] = PlaneError(plane, plane_gt, param)

    nError = 0;
    dError = 0;

    for plane_i = 1:param.plane_num
        n = plane.normal_vector_list{1, plane_i};
        d = plane.q_list{1, plane_i};
        n_gt = plane_gt.normal_vector_gt_list{1, plane_i};
        d_gt = -n_gt' * plane_gt.q_gt_list{1, plane_i};
        nError = nError + norm(abs(n) - abs(n_gt))^2;
        dError = dError + norm(abs(d) - abs(d_gt))^2;
    end
    
    nError = nError / param.plane_num;
    dError = dError / param.plane_num;

end