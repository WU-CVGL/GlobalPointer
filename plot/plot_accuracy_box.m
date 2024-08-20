function plot_accuracy_box(results, method_list, plot_list, x_axis_list, title_list, is_log10, y_axis_title, x_axis_title)
    
    epoch_size_num = size(results{1, 1}.error, 1);
    iteration_size_num = size(results{1, 1}.error, 2);
    error_name = fieldnames(results{1, 1}.error{1, 1});
    error_size_num = size(error_name, 1);
    method_size_num = size(plot_list, 2);

    error_size_num = 5;

    for error_i = 1:error_size_num
        x_list = [];
        y_list = [];
        label_list = [];

        for plot_i = 1:method_size_num
            x_tmp_list = [];
            y_tmp_list = [];

            plot_method = plot_list(1, plot_i);
            method_id = find(method_list == plot_method);
            error_all = results{method_id, 1}.error;
            
            for epoch_i = 1:epoch_size_num
                error_single = [];
                for iteration_i = 1:iteration_size_num
                    if ~isempty(error_all{epoch_i, iteration_i})
                        error_struct = error_all{epoch_i, iteration_i};
                        error_cell = struct2cell(error_struct);
                        error_cell = error_cell{error_i, 1};
                        error_single = [error_single;error_cell(1, end)];
                    end
                end
                if is_log10
                    y_tmp_list = [y_tmp_list; log10(error_single)];
                else
                    y_tmp_list = [y_tmp_list; (error_single)];
                end
                x_tmp_list = [x_tmp_list; repmat(x_axis_list(1, epoch_i), size(error_single))];
            end
            x_list = [x_list; x_tmp_list];
            y_list = [y_list; y_tmp_list];
            label_list = [label_list;repmat(plot_method, size(x_tmp_list))];
            
        end

        x_list = categorical(x_list, x_axis_list);
        figure
        boxchart(x_list,y_list,'GroupByColor',label_list);
        title(title_list(1, error_i));
        xlabel(x_axis_title);
        ylabel(y_axis_title);
        legend;
        set(gcf, 'color', 'w');

    end
        
end