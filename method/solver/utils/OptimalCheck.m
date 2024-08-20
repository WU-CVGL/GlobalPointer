function flag = OptimalCheck(total_error_list, runtime_param)
    flag = true;
    if size(total_error_list, 2) > 3 && total_error_list(1, end) - total_error_list(1, end - 1) > 0
        if abs(total_error_list(1, end) - total_error_list(1, end - 1)) > total_error_list(1, end)
            flag = false;
        end
    end
end