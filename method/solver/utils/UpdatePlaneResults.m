function [plane_estimate_struct,plane_estimate_matrix,optimal_plane_vec] = UpdatePlaneResults(X_list, param)

    
    optimal_plane_vec = ones(1, param.plane_num);
    for plane_i = 1:param.plane_num
        [U,S,V] = svd(X_list{1,plane_i});
        % [d,ind] = sort(diag(S));
        % Ss = S(ind,ind);
        % Vs = V(:,ind);
        % 
        % A = sqrt(Ss)*Vs';
        % x_solution = A(4, :);
        % x_solution = x_solution / norm(x_solution(1, 1:3));

        A = S*V';
        x_solution = A(1, :);
        x_solution = x_solution / norm(x_solution(1, 1:3));

        plane_estimate_struct.normal_vector_list{1, plane_i} = x_solution(1:3)';
        plane_estimate_struct.q_list{1, plane_i} = x_solution(4);

        % if Ss(1,1) / Ss(2,2) < 9
        %     optimal_plane_vec(1, plane_i) = 0;
        % end
    end


    plane_estimate_matrix = zeros(4, param.plane_num);
    for plane_i = 1:param.plane_num
        plane_estimate_matrix(:, plane_i) = [plane_estimate_struct.normal_vector_list{plane_i}; plane_estimate_struct.q_list{plane_i}];
    end
end