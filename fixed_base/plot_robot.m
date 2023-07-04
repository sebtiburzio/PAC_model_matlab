function plot_robot(x_ev,type)

global p_vals

%     close all

    n_step = x_ev.time(end)*30;
    stop_time = x_ev.time(end);
    time_vect = linspace(0,stop_time,n_step);
    
    x_ev_res = resample(x_ev,time_vect);
    
    s_vect = 0:0.1:1;
    for i_t = 1:length(time_vect)
        xy_c = nan(length(s_vect),2);
        for i_s = 1:length(s_vect)
            xy_c(i_s,:) = fk_fcn(p_vals', [x_ev_res.Data(i_t,1),x_ev_res.Data(i_t,2)]', s_vect(i_s), 0);
        end

        if type == 1
            % overlay plot
            hold on, plot(xy_c(:,1),xy_c(:,2), 'linewidth', 0.4, 'color', [0.9,0.9,0.9]-(0.89*i_t/n_step))
        elseif type == 2
            % animated plot
            plot(xy_c(:,1),xy_c(:,2), 'linewidth', 0.4, 'color', [0,0,0])
            title(time_vect(i_t))
        end

%         xlim([xy_c(1,1)-(p_vals(3)+0.1),xy_c(1,1)+(p_vals(3)+0.1)])
%         ylim([xy_c(1,2)-(p_vals(3)+0.1),xy_c(1,2)+(p_vals(3)+0.1)])
        xlim([-0.6,0.6])
%         ylim([-0.8,0.2])
        axis equal
        grid on

        drawnow
        pause(0.01)

    end
    
    return