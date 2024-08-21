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

function [F] = F_AutoConstruct(x, opt_func, ObsSet, parallel_calculation_flag)

    obs_num = size(ObsSet, 2);
    [F_tmp] = opt_func(x, ObsSet{1, 1});
    error_dim = size(F_tmp, 1);
    parameter_num = size(x, 2);

    if parallel_calculation_flag
        F = zeros(error_dim, obs_num);
        parfor obs_i = 1:obs_num
            [F_tmp] = opt_func(x, ObsSet{1, obs_i});
            F(:, obs_i) = F_tmp;
        end
    else
        F = zeros(error_dim, obs_num);
        for obs_i = 1:obs_num
            [F_tmp] = opt_func(x, ObsSet{1, obs_i});
            F(:, obs_i) = F_tmp;
        end
    end
    
end