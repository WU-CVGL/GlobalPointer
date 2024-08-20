function [F, J] = F_J_AutoConstruct(x, opt_func, ObsSet, parallel_calculation_flag)
    obs_num = size(ObsSet, 2);
    [F_tmp, J_tmp] = opt_func(x, ObsSet{1, 1});
    error_dim = size(F_tmp, 1);
    parameter_num = size(x, 2);



    if parallel_calculation_flag
        F = zeros(error_dim, obs_num);
        J = cell(obs_num, 1);
        parfor obs_i = 1:obs_num
            [F_tmp, J_tmp] = opt_func(x, ObsSet{1, obs_i});
            F(:, obs_i) = F_tmp;
            J{obs_i, 1} = J_tmp;
        end
        J = cell2mat(J);
    else
        F = zeros(error_dim, obs_num);
        J = zeros(obs_num * error_dim, parameter_num);
        for obs_i = 1:obs_num
            [F_tmp, J_tmp] = opt_func(x, ObsSet{1, obs_i});
            F(:, obs_i) = F_tmp;
            J_id = (obs_i - 1) * error_dim;
            J(J_id+1:J_id+error_dim, :) = J_tmp;
        end

    end
    
    

end
