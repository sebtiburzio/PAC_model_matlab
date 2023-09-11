%%
clear
clear global
rmpath('../fixed_base/automatically_generated')
addpath('automatically_generated')

%%
t = tiledlayout(2,3,'TileSpacing','Tight');

%%
Endpt_Meas = [X_end_meas(1:90) Z_end_meas(1:90)]';
Endpt_Goals = [Goal_X(1:90) Goal_Z(1:90)]';
Endpt_Sols = [Endpt_Sol_X(1:90) Endpt_Sol_Z(1:90)]';
ts = ts(1:90);
nX = 15;
nY = length(ts)/nX;

model_error = reshape(vecnorm(Endpt_Goals - Endpt_Sols),[nX,nY])';
endpt_error = reshape(vecnorm(Endpt_Goals - Endpt_Meas),[nX,nY])';

xgrid_heatmap = -0.75:0.1:0.75;
zgrid_heatmap = 0.0:0.1:0.1*nY;

plot_clr = [1 1 1]; % Colour for lines between goal/modelled and measured points

%% Plots
topplt = 3; % Model comparison, control/nophicost/phicost, 1/2/3
btmplt = 6; % Measured comparison, control/nophicost/phicost, 4/5/6
% Model endpoints compared to goal points
nexttile(topplt)
pcolor(xgrid_heatmap,zgrid_heatmap,[model_error zeros(nY,1); zeros(1,nX) 0])
caxis([0.0 0.2])
axis equal
xlim([-0.75,0.75]);
xticks(-0.6:0.2:0.6)
xticklabels('auto')
ylim([0 0.1*nY]);
hold on
for i = 1:length(ts)
    plot([Endpt_Sols(1,i) Endpt_Goals(1,i)], [Endpt_Sols(2,i) Endpt_Goals(2,i)],'--','color',plot_clr,'linewidth',1.5)
end
scatter(Endpt_Sols(1,:),Endpt_Sols(2,:),20,plot_clr,'filled');
ax = gca;
ax.FontSize = 14;

% Measured endpoints compared to goal points (nexttile 4,5,6)
nexttile(btmplt)
pcolor(xgrid_heatmap,zgrid_heatmap,[endpt_error zeros(nY,1); zeros(1,nX) 0])
caxis([0.0 0.2])
axis equal
xlim([-0.75,0.75]);
xticks(-0.6:0.2:0.6)
xticklabels('auto')
ylim([0 0.1*nY]);
xlabel('x_B (m)')
hold on
for i = 1:length(ts)
    plot([Endpt_Meas(1,i) Endpt_Sols(1,i)], [Endpt_Meas(2,i) Endpt_Sols(2,i)],':','color',plot_clr,'linewidth',1.5)
end
scatter(X_end_meas(:),Z_end_meas(:),20,plot_clr,'filled');
ax = gca;
ax.FontSize = 14;

%% Only ylabel on leftmost plots
nexttile(1)
ylabel('y_B (m)')
nexttile(4)
ylabel('y_B (m)')

%% Only colourbar on rightmost plots
nexttile(3)
cax = colorbar;
cax.Label.String = '\Delta p_e (m)';
nexttile(6)
cax = colorbar;
cax.Label.String = '\Delta p_e (m)';
caxis([0.0 0.2])

%%
% Prep data - for black weighted cable case (except control) collected differently (date 0814 & 0815)
% for 0814/0815 experiments goals/model solutions were saved as .mat, other dates these were piped through experiment so all available in one csv file. Hopefully I or no on else has to try to work with this data again.
nX = 15;
nY = 6;

xgrid_meas = -0.7:0.1:0.7;
zgrid_meas = 0.05:0.1:0.55;
xgrid_heatmap = -0.75:0.1:0.75;
zgrid_heatmap = 0.0:0.1:0.6;

Endpt_Meas = [X_end_meas Z_end_meas]';
Endpt_Goals = Goals; 

model_endpts = [];
for i = 1:length(Sequence)
    model_endpts = [model_endpts fk_fcn(p_vals,[Curvature(:,i); Sequence(:,i)],1,0)];
end
Endpt_Sols = [model_endpts(1,:); model_endpts(2,:)];
model_error = reshape(vecnorm(Endpt_Goals - Endpt_Sols),[nX,nY])';
endpt_error = reshape(vecnorm(Endpt_Goals - Endpt_Meas),[nX,nY])';

plot_clr = [1 1 1]; % Colour for lines between goal/modelled and measured points

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

%% Orientation Experiment
% Note the names are all messed up, Goal_Phi is the goal for the tip angle (alpha), and Base_angle is the measured tip angle. Sad.

% Angle error plot
figure
scatter(-Goal_Phi(1:13),Goal_Phi(1:13)-Base_angle(1:13),60,[0 0.4470 0.7410],'filled')
xlabel('\psi_{g,B^\prime} (rad)')
xlim([-1.7,1.7])
xticks(-pi:pi/4:pi)
xticklabels({'-\pi','-3\pi/4','-\pi/2','-\pi/4','0','\pi/4','\pi/2','3\pi/4','\pi'})
ylabel('\Delta \psi_{e,B^\prime} (rad)')
ylim([-0.15,0.3])

grid on
box on
ax = gca;
ax.FontSize = 24;

%%
exportgraphics(ax,'D:\Study\Thesis\Report\images\orientation_analysis\orange_orientation_error.eps')

%% Angle visualisation
figure
hold on
for i = 1:13 % between -pi/2 and pi/2, 15 deg increments
%     plot([0 cos(Goal_Phi(i))],[0 sin(Goal_Phi(i))],'g')
    plot([0 cos(Base_angle(i))],[0 sin(Base_angle(i))],'r')
    plot([cos(Goal_Phi(i)) cos(Base_angle(i))], [sin(Goal_Phi(i)) sin(Base_angle(i))], 'k:')
end
axis equal
hold off

%% Trace of endpoint, normalised to goal at (0,0)
plot(X_end_meas(1:13),Z_end_meas(1:13)-0.45)