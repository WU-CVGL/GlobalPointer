function [g] = NumericalGradient(x, opt_func, update_func, parallel_flag)
    parameter_dim = size(x, 2);
    g = zeros(1, parameter_dim);

    if parallel_flag
        
    else
        for i = 1:parameter_dim
            % --- forward ---
            x_p = zeros(1, parameter_dim);
            x_p(1, i) = 1e-6;
            x_tmp = update_func(x, x_p);
            [F_p] = opt_func(x_tmp);

            % --- backward ---
            x_n = zeros(1, parameter_dim);
            x_n(1, i) = -1e-6;
            x_tmp = update_func(x, x_n);
            [F_n] = opt_func(x_tmp);

            j_slice = (F_p - F_n) / 2e-6;
            g(:, i) = j_slice;
        end
    end
    
    

end
