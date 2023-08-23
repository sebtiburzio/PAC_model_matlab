%%
clear
addpath('automatically_generated')

%% Prep data
xgrid_meas = -0.7:0.1:0.7;
zgrid_meas = 0.05:0.1:0.55;
xgrid_heatmap = -0.75:0.1:0.75;
zgrid_heatmap = 0.0:0.1:0.6;
GR_cmap = [interp1([0;1],[0 1 0; 1 1 0],linspace(0,1,128)); interp1([0;1],[1 1 0; 1 0 0],linspace(0,1,128))];

model_endpts = [];
for i = 1:length(Sequence)
    model_endpts = [model_endpts fk_fcn(p_vals,[Curvature(:,i); Sequence(:,i)],1,0)];
end
model_error = reshape(vecnorm(Goals - [model_endpts(1,:); model_endpts(2,:)]),[15,6])';

endpt_error = reshape(vecnorm(Goals - [X_end_meas'; Z_end_meas']),[15,6])';

%% Plots
% Model endpoints compared to goal points
plot_clr = [1 1 1];
subplot(2,1,1)
pcolor(xgrid_heatmap,zgrid_heatmap,[model_error zeros(6,1); zeros(1,15) 0])
% colormap(gca,GR_cmap);
axis equal
xlim([-0.75,0.75]);
ylim([0 0.6]);
ylabel('y_B (m)')
cax = colorbar;
cax.Label.String = '\Delta p_e (modelled) (m)';
caxis([0.0 0.2])
hold on
for i = 1:length(Sequence)
    plot([model_endpts(1,i) Goals(1,i)], [model_endpts(2,i) Goals(2,i)],'--','color',plot_clr,'linewidth',1.5)
end
scatter(model_endpts(1,:),model_endpts(2,:),20,plot_clr,'filled');

% Measured endpoints compared to goal points 
subplot(2,1,2)
pcolor(xgrid_heatmap,zgrid_heatmap,[endpt_error zeros(6,1); zeros(1,15) 0])
% colormap(gca,GR_cmap);
axis equal
xlim([-0.75,0.75]);
ylim([0 0.6]);
xlabel('x_B (m)')
ylabel('y_B (m)')
cax = colorbar;
cax.Label.String = '\Delta p_e (measured) (m)';
caxis([0.0 0.2])
hold on
for i = 1:length(Sequence)
    plot([X_end_meas(i) model_endpts(1,i)], [Z_end_meas(i) model_endpts(2,i)],':','color',plot_clr,'linewidth',1.5)
end
scatter(X_end_meas(:),Z_end_meas(:),20,plot_clr,'filled');


%% 
% Special for control (no model) case where some points omitted
model_error = vecnorm(Goals - [model_endpts(1,:); model_endpts(2,:)]);
endpt_error = vecnorm(Goals - [X_end_meas'; Z_end_meas']);
% Fill in first/last 2 points of row 1, first/last 1 point of row 2
model_error = [NaN NaN model_error(1:11) NaN NaN NaN model_error(12:24) NaN model_error(25:end)]; 
endpt_error = [NaN NaN endpt_error(1:11) NaN NaN NaN endpt_error(12:24) NaN endpt_error(25:end)];
% Add two extra rows
model_error = [model_error NaN*ones(1,30)];
endpt_error = [endpt_error NaN*ones(1,30)];

model_error = reshape(model_error,[15,6])';
endpt_error = reshape(endpt_error,[15,6])';

