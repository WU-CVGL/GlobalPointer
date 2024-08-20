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
