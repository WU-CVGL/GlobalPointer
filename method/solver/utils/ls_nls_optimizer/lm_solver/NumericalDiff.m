function [J] = NumericalDiff(x, F, opt_func, update_func, parallel_flag)
    observation_num = size(F, 2);
    observation_dim = size(F, 1);
    parameter_dim = size(x, 2);
    J = zeros(observation_dim * observation_num, parameter_dim);

    if parallel_flag
        
    else
        for i = 1:parameter_dim
            % --- forward ---
            x_p = zeros(1, parameter_dim);
            x_p(1, i) = 1e-6;
            x_tmp = update_func(x, x_p);
            [F_p] = opt_func(x_tmp);
            F_p = reshape(F_p, [observation_dim * observation_num, 1]);

            % --- backward ---
            x_n = zeros(1, parameter_dim);
            x_n(1, i) = -1e-6;
            x_tmp = update_func(x, x_n);
            [F_n] = opt_func(x_tmp);
            F_n = reshape(F_n, [observation_dim * observation_num, 1]);

            j_slice = (F_p - F_n) / 2e-6;
            J(:, i) = j_slice';
        end
    end
    
    

end
