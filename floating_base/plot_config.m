function plot_config(q,alpha)

global p_vals

s_vect = 0:0.05:1;
xy_c = nan(length(s_vect),2);
for i_s = 1:length(s_vect)
    xy_c(i_s,:) = fk_fcn(p_vals, q, s_vect(i_s), 0);
end

plot([xy_c(1,1),xy_c(1,1)+0.05*sin(q(5))],[xy_c(1,2),xy_c(1,2)+0.05*cos(q(5))],'k',LineWidth=3)
hold on
% plot([0,0],[0,0.333],'k', LineWidth=3) % Robot base to joint1
plot(xy_c(:,1),xy_c(:,2), Color=[1.0 1.0-alpha*0.5 1.0-alpha*1.0], LineWidth=3)
scatter(xy_c(11,1),xy_c(11,2), 20, [0.4660 0.6740 0.1880], 'filled')
scatter(xy_c(end,1),xy_c(end,2), 20, [0 0.4470 0.7410], 'filled')
xlim([xy_c(1,1)-p_vals(3)-0.1,xy_c(1,1)+p_vals(3)+0.1])
axis equal
grid on
hold off