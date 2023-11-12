function plot_robot(x_ev, type)

global p_vals

%     close all

%     f = figure;  
%     f.Position = [100 100 625 400]; 

    n_step = x_ev.time(end)/(1/30);
    stop_time = x_ev.time(end);%6;%1.3;
    time_vect = linspace(0,stop_time,n_step);
    
    x_ev_res = resample(x_ev,time_vect);
    
    s_vect = 0:0.1:1;
    for i_t = 1:length(time_vect)
        xy_c = nan(length(s_vect),2);
        for i_s = 1:length(s_vect)
            xy_c(i_s,:) = fk_fcn(p_vals, ...
                                 [x_ev_res.Data(i_t,1),x_ev_res.Data(i_t,2), ...
                                  x_ev_res.Data(i_t,3),x_ev_res.Data(i_t,4),x_ev_res.Data(i_t,5)]', ...
                                 s_vect(i_s), 0);
        end

        if type == 1
            % overlay plot
            hold on, plot(xy_c(:,1),xy_c(:,2), 'linewidth', 0.4, 'color', [0.9,0.9,0.9]-(0.89*i_t/n_step))
        elseif type == 2
            % animated plot
            plot([xy_c(1,1),xy_c(1,1)+0.05*sin(x_ev_res.Data(i_t,5))],[xy_c(1,2),xy_c(1,2)+0.05*cos(x_ev_res.Data(i_t,5))],'k',LineWidth=3)
            hold on
            plot(xy_c(:,1),xy_c(:,2), Color=[1.0 0.5 0.0], LineWidth=3)
            hold off
            title(time_vect(i_t))
        end

        xlim([xy_c(1,1)-(p_vals(3)+0.1),xy_c(1,1)+(p_vals(3)+0.1)])
        ylim([xy_c(1,2)-(p_vals(3)+0.1),xy_c(1,2)+0.1])
%         xlim([-0.6,0.6])
%         ylim([-0.8,0.2])
        axis equal
        grid on

        drawnow
        % Export frames for animation
        % ax = gca;
        % exportgraphics(ax,'./frames/' + string(i_t) + '.png','Resolution',300)
        pause(0.01) % TODO - use rateControl to plot in real time rate?

    end
    
    return