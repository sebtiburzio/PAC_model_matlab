
% load in the old figures (not clear how to get the original data for each figure)
old_ob1_fig = openfig("../data_in/0802-equilibrium_data/black_weighted_equilibria/black_weighted_endpt_err.fig");
old_ob2_fig = openfig("../data_in/0802-equilibrium_data/black_unweighted_equilibria/black_unweighted_endpt_err.fig");
old_ob3_fig = openfig("../data_in/0802-equilibrium_data/black_short_weighted_equilibria/black_short_weighted_endpt_err.fig");
old_ob4_fig = openfig("../data_in/0802-equilibrium_data/black_short_unweighted_equilibria/black_short_unweighted_endpt_err.fig");
old_ob5_fig = openfig("../data_in/0802-equilibrium_data/orange_weighted_equilibria/orange_weighted_endpt_err.fig");
old_ob6_fig = openfig("../data_in/0802-equilibrium_data/orange_short_unweighted_equilibria/orange_short_unweighted_endpt_err.fig");


% load in the object parameters to the length of each object
p_vals_ob1 = load('../../object_parameters/black_weighted.mat');
p_vals_ob2 = load('../../object_parameters/black_unweighted.mat');
p_vals_ob3 = load('../../object_parameters/black_short_weighted.mat');
p_vals_ob4 = load('../../object_parameters/black_short_unweighted.mat');
p_vals_ob5 = load('../../object_parameters/orange_weighted.mat');
p_vals_ob6 = load('../../object_parameters/orange_short_unweighted.mat');


% open('2.fig');
% h = gcf; %current figure handle
axesObjs_ob1 = get(old_ob1_fig, 'Children');  %axes handles
axesObjs_ob2 = get(old_ob2_fig, 'Children');  %axes handles
axesObjs_ob3 = get(old_ob3_fig, 'Children');  %axes handles
axesObjs_ob4 = get(old_ob4_fig, 'Children');  %axes handles
axesObjs_ob5 = get(old_ob5_fig, 'Children');  %axes handles
axesObjs_ob6 = get(old_ob6_fig, 'Children');  %axes handles

dataObjs_ob1 = get(axesObjs_ob1, 'Children'); %handles t
dataObjs_ob2 = get(axesObjs_ob2, 'Children'); %handles t
dataObjs_ob3 = get(axesObjs_ob3, 'Children'); %handles t
dataObjs_ob4 = get(axesObjs_ob4, 'Children'); %handles t
dataObjs_ob5 = get(axesObjs_ob5, 'Children'); %handles t
dataObjs_ob6 = get(axesObjs_ob6, 'Children'); %handles t



xdata_ob1 = get(dataObjs_ob1, 'XData'); 
xdata_ob2 = get(dataObjs_ob2, 'XData'); 
xdata_ob3 = get(dataObjs_ob3, 'XData'); 
xdata_ob4 = get(dataObjs_ob4, 'XData'); 
xdata_ob5 = get(dataObjs_ob5, 'XData'); 
xdata_ob6 = get(dataObjs_ob6, 'XData'); 

xdata_ob1 = xdata_ob1(end, :);
xdata_ob2 = xdata_ob2(end, :);
xdata_ob3 = xdata_ob3(end, :);
xdata_ob4 = xdata_ob4(end, :);
xdata_ob5 = xdata_ob5(end, :);
xdata_ob6 = xdata_ob6(end, :);


ydata_ob1 = get(dataObjs_ob1, 'YData');
ydata_ob2 = get(dataObjs_ob2, 'YData');
ydata_ob3 = get(dataObjs_ob3, 'YData');
ydata_ob4 = get(dataObjs_ob4, 'YData');
ydata_ob5 = get(dataObjs_ob5, 'YData');
ydata_ob6 = get(dataObjs_ob6, 'YData');

ydata_ob1_l = ydata_ob1(end, :);
ydata_ob2_l = ydata_ob2(end, :);
ydata_ob3_l = ydata_ob3(end, :);
ydata_ob4_l = ydata_ob4(end, :);
ydata_ob5_l = ydata_ob5(end, :);
ydata_ob6_l = ydata_ob6(end, :);


% calculate the % endpt error
ydata_ob1_r = 100 * ydata_ob1_l{1}/p_vals_ob1.p_vals(3);
ydata_ob2_r = 100 * ydata_ob2_l{1}/p_vals_ob2.p_vals(3);
ydata_ob3_r = 100 * ydata_ob3_l/p_vals_ob3.p_vals(3);
ydata_ob4_r = 100 * ydata_ob4_l/p_vals_ob4.p_vals(3);
ydata_ob5_r = 100 * ydata_ob5_l{1}/p_vals_ob5.p_vals(3);
ydata_ob6_r = 100 * ydata_ob6_l/p_vals_ob6.p_vals(3);


% Save data as .mat files
ob1_endpt_err = [xdata_ob1; ydata_ob1_l];
% save('../../../../julia/data/endpt_error/ob1_endpt_err.mat', 'ob1_endpt_err');
csvwrite('../../../../julia/data/endpt_error/ob1_endpt_err.csv', ob1_endpt_err);

ob1_endpt_err_percent = [xdata_ob1; ydata_ob1_r];
csvwrite('../../../../julia/data/endpt_error/ob1_endpt_err_percent.csv', ob1_endpt_err_percent);


ob2_endpt_err = [xdata_ob2; ydata_ob2_l];
% save('../../../../julia/data/endpt_error/ob2_endpt_err.mat', 'ob2_endpt_err');
csvwrite('../../../../julia/data/endpt_error/ob2_endpt_err.csv', ob2_endpt_err);
ob2_endpt_err_percent = [xdata_ob2; ydata_ob2_r];
% save('../../../../julia/data/endpt_error/ob2_endpt_err_percent.mat', 'ob2_endpt_err_percent');
csvwrite('../../../../julia/data/endpt_error/ob2_endpt_err_percent.csv', ob2_endpt_err_percent);


ob3_endpt_err = [xdata_ob3; ydata_ob3_l];
% save('../../../../julia/data/endpt_error/ob3_endpt_err.mat', 'ob3_endpt_err');
csvwrite('../../../../julia/data/endpt_error/ob3_endpt_err.csv', ob3_endpt_err);
ob3_endpt_err_percent = [xdata_ob3; ydata_ob3_r];
% save('../../../../julia/data/endpt_error/ob3_endpt_err_percent.mat', 'ob3_endpt_err_percent');
csvwrite('../../../../julia/data/endpt_error/ob3_endpt_err_percent.csv', ob3_endpt_err_percent);

ob4_endpt_err = [xdata_ob4; ydata_ob4_l];
% save('../../../../julia/data/endpt_error/ob4_endpt_err.mat', 'ob4_endpt_err');
csvwrite('../../../../julia/data/endpt_error/ob4_endpt_err.csv', ob4_endpt_err);
ob4_endpt_err_percent = [xdata_ob4; ydata_ob4_r];
% save('../../../../julia/data/endpt_error/ob4_endpt_err_percent.mat', 'ob4_endpt_err_percent');
csvwrite('../../../../julia/data/endpt_error/ob4_endpt_err_percent.csv', ob4_endpt_err_percent);

ob5_endpt_err = [xdata_ob5; ydata_ob5_l];
% save('../../../../julia/data/endpt_error/ob5_endpt_err.mat', 'ob5_endpt_err');
csvwrite('../../../../julia/data/endpt_error/ob5_endpt_err.csv', ob5_endpt_err);
ob5_endpt_err_percent = [xdata_ob5; ydata_ob5_r];
% save('../../../../julia/data/endpt_error/ob5_endpt_err_percent.mat', 'ob5_endpt_err_percent');
csvwrite('../../../../julia/data/endpt_error/ob5_endpt_err_percent.csv', ob5_endpt_err_percent);

ob6_endpt_err = [xdata_ob6; ydata_ob6_l];
% save('../../../../julia/data/endpt_error/ob6_endpt_err.mat', 'ob6_endpt_err');
csvwrite('../../../../julia/data/endpt_error/ob6_endpt_err.csv', ob6_endpt_err);
ob6_endpt_err_percent = [xdata_ob6; ydata_ob6_r];
% save('../../../../julia/data/endpt_error/ob6_endpt_err_percent.mat', 'ob6_endpt_err_percent');
csvwrite('../../../../julia/data/endpt_error/ob6_endpt_err_percent.csv', ob6_endpt_err_percent);
% Get the vector of endpt errors for each object
% ob1_endpt_err = old_ob1_fig.CurrentAxes.Children.YData;
% ob2_endpt_err = old_ob2_fig.CurrentAxes.Children.YData;
% ob3_endpt_err = old_ob3_fig.CurrentAxes.Children.YData;
% ob4_endpt_err = old_ob4_fig.CurrentAxes.Children.YData;
% ob5_endpt_err = old_ob5_fig.CurrentAxes.Children.YData;
% ob6_endpt_err = old_ob6_fig.CurrentAxes.Children.YData;

% Plot the % endpt error for each plot
% hold on
% yyaxis(old_ob1_fig.CurrentAxes,'right');

% Make changes to ob1 figure
ylim_l = axesObjs_ob1.YLim(2);
y_max_l = max(ydata_ob1_l{1});
alpha = y_max_l/ylim_l;
y_max_r = max(ydata_ob1_r);
ylim_r = alpha * y_max_r;

yyaxis(old_ob1_fig.CurrentAxes,'right');
% axesObjs_ob1.YAxisLocation = 'right';

axesObjs_ob1.YAxisLocation;
% axesObjs_ob1.YLimMode = 'manual'
% axesObjs_ob1.YLim = 'manual'

hold on 
axesObjs_ob1.YLim = [0, ylim_r];
axesObjs_ob1.YLim
axesObjs_ob1.YLimMode
axesObjs_ob1.YLimMode = 'manual'
axesObjs_ob1.YLimitMethod
axesObjs_ob1.YLimitMethod = 'tickaligned'
axesObjs_ob1.YTick(end) = ylim_r
% ylim manual 
plot(axesObjs_ob1, xdata_ob1{1}, ydata_ob1_r, 'o'); ylim([0 0.06])
axesObjs_ob1.YLimMode
axesObjs_ob1.YLim
axesObjs_ob1.YLimitMethod
% hold off


% hold on
yyaxis(old_ob2_fig.CurrentAxes,'right');
scatter(axesObjs_ob2, xdata_ob2{1}, ydata_ob2_r)
% hold off

% hold on
% yyaxis(old_ob3_fig.CurrentAxes,'right');
% scatter(axesObjs_ob3, xdata_ob3, ydata_ob3_r)
% hold off

% hold on
% yyaxis(old_ob4_fig.CurrentAxes,'right');
% scatter(axesObjs_ob4, xdata_ob4, ydata_ob4_r)
% hold off

% hold on
% yyaxis(old_ob5_fig.CurrentAxes,'right');
% scatter(axesObjs_ob5, xdata_ob5{1}, ydata_ob5_r)
% hold off

% yyaxis(old_ob6_fig.CurrentAxes,'right');
% scatter(axesObjs_ob6, xdata_ob6, ydata_ob6_r)
