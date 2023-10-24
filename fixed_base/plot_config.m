function plot_config(theta0,theta1,alpha,rotang)

global p_vals

s_vect = 0:0.05:1;
xy_c = nan(length(s_vect),2);
for i_s = 1:length(s_vect)
    xy_c(i_s,:) = fk_fcn(p_vals, [theta0,theta1]', s_vect(i_s), 0);
end

xy_c = ([cos(rotang) sin(rotang); -sin(rotang) cos(rotang)]*xy_c')';

plot(xy_c(:,1),xy_c(:,2), Color=[1.0 1.0-alpha*0.5 1.0-alpha*1.0], LineWidth=3)
hold on
% plot([-0.05,0.05],[0,0],'k',LineWidth=2)
scatter(xy_c(11,1),xy_c(11,2), 30, [0.4660 0.6740 0.1880], 'filled')
scatter(xy_c(end,1),xy_c(end,2), 30, [0 0.4470 0.7410], 'filled')
xlim([-p_vals(3)-0.1,p_vals(3)+0.1])
axis equal
xlabel('x (m)')
ylabel('y (m)')

grid on
hold off