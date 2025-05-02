%%
clear
clear global
rmpath('../../fixed_base/automatically_generated')
addpath('../automatically_generated')

%%
t = tiledlayout(2,3,'TileSpacing','Tight');

%% Load in the csv file for the first object and condition

% load('..\data_in\0829-static_wkspaceCD\black_short_weighted_control.csv')
% readtable('../data_in/0829-static_wkspaceCD/black_short_weighted_control.csv')
% [ts, X_EE, Y_EE, Z_EE, Phi, X_base_meas, Z_base_meas, X_mid_meas, Z_mid_meas, X_end_meas, Z_end_meas, U_base, V_base, U_mid, V_mid, U_end, V_end, Base_angle, X_and_start, Z_ang_start, X_ang_end, Z_ang_end, U_ang_start, V_ang_start, U_and_end, V_ang_end, Goal_X, Goal_Z, Endpt_Sol_X, Endpt_Sol_Z] = csvimport('../data_in/0829-static_wkspaceCD/black_short_weighted_control.csv', 'columns');
data_path = '../data_in/0829-static_wkspaceCD/black_short_weighted_control.csv';

% data = csvread('../data_in/0829-static_wkspaceCD/black_short_weighted_control.csv');
data = csvread(data_path, 2);
%
% % Get the column names to be used as variable names
% fid      = fopen(data_path);
% paramIds = textscan(fid,'%s',1,'HeaderLines',0);
% fclose(fid);
ts = data(:,1); 
X_EE = data(:,2); 
Y_EE = data(:,3); 
Z_EE = data(:,4); 
Phi = data(:,5); 
X_base_meas = data(:,6); 
Z_base_meas = data(:,7); 
X_mid_meas = data(:,8); 
Z_mid_meas = data(:,9); 
X_end_meas = data(:, 10);
Z_end_meas = data(:, 11); 
U_base = data(:, 12); 
V_base = data(:, 13);
U_mid = data(:,14); 
V_mid = data(:,15); 
U_end = data(:, 16);
V_end = data(:, 17); 
Base_angle = data(:, 18); 
X_and_start = data(:, 19); 
Z_ang_start = data(:, 20); 
X_ang_end = data(:, 21); 
Z_ang_end = data(:, 22); 
U_ang_start = data(:, 23); 
V_ang_start = data(:, 24); 
U_and_end = data(:, 25); 
V_ang_end = data(:, 26); 
Goal_X = data(:, 27); 
Goal_Z = data(:, 28); 
Endpt_Sol_X = data(:, 29); 
Endpt_Sol_Z = data(:, 30);

%Import the options of the csv file
opts=detectImportOptions(data_path);
%Defines the row location of channel variable name
opts.VariableNamesLine = 1;
%Specifies that the data is comma seperated
opts.Delimiter =','; %Specifies that the data is comma seperated
%Read the table
data = readtable(data_path, opts, 'ReadVariableNames', true);

%%
Endpt_Meas = [X_end_meas(:) Z_end_meas(:)]';
Endpt_Goals = [Goal_X(:) Goal_Z(:)]';
Endpt_Sols = [Endpt_Sol_X(:) Endpt_Sol_Z(:)]';
ts = ts(:);
nX = 15;
nY = length(ts)/nX;

model_error = reshape(vecnorm(Endpt_Goals - Endpt_Sols),[nX,nY])';
endpt_error = reshape(vecnorm(Endpt_Goals - Endpt_Meas),[nX,nY])';

xgrid_heatmap = -0.75:0.1:0.75;
zgrid_heatmap = 0.0:0.1:0.1*nY;

plot_clr = [1 1 1]; % Colour for lines between goal/modelled and measured points

%% Statistics
% mean(model_error,'all')
% median(model_error,'all')
% mean(endpt_error,'all')
% median(endpt_error,'all')

%% Error mean plot without phi cost case

% load('..\data_in\0829-static_wkspaceCD\endpt_error_means_without_phi_cost_case.mat')
% ob_cats = categorical({'OB5','OB3','OB2','OB1'});
% ob_cats = reordercats(ob_cats,{'OB5','OB3','OB2','OB1'});
% b = barh(flip(ob_cats,2),flip(endpt_error_means_without_phi_cost_case,2));
% b(1).FaceColor = [0.3010 0.7450 0.9330];
% b(2).FaceColor = [0 0.262 0.5];
% b(3).FaceColor = [0.9290 0.6940 0.1250];
% b(4).FaceColor = [0.6350 0.0780 0.1840];
% xlabel('Mean $\Delta p_e (m)$', 'Interpreter','latex')
% ax = gca;
% set(ax, 'FontSize', 18)
% set(ax, 'TickLabelInterpreter', 'latex')
% grid on
% box on
%
% bar_labels=categorical({'Reference (modeled)','Using controller (modeled)','Reference (measured)','Using controller (measured)'});
% bar_labels=reordercats(bar_labels,{'Reference (modeled)','Using controller (modeled)','Reference (measured)','Using controller (measured)'});
% hold on
% bh(1) = barh(nan,nan,'FaceColor',[0.6350 0.0780 0.1840]);
% bh(2) = barh(nan,nan,'FaceColor',[0.9290 0.6940 0.1250]);
% bh(3) = barh(nan,nan,'FaceColor',[0 0.262 0.5]);
% bh(4) = barh(nan,nan,'FaceColor',[0.3010 0.7450 0.9330]);
% xlim([0,0.25])
% legend(bh, string(bar_labels),'Location','northoutside')
% l = findobj(gcf, 'Type', 'legend');
% set(l, 'Interpreter', 'latex')
% set(gcf, 'Position', [833 220 466 611])
%
% %% Error mean plot without phi cost case - horizontal orientation
%
% load('..\data_in\0829-static_wkspaceCD\endpt_error_means.mat')
% ob_cats = categorical({'OB1','OB2','OB3','OB5'});
% ob_cats = reordercats(ob_cats,{'OB1','OB2','OB3','OB5'});
% b = bar(ob_cats,endpt_error_means(:,[1,2,4,5]));
% b(1).FaceColor = [0.6350 0.0780 0.1840];
% b(2).FaceColor = [0.9290 0.6940 0.1250];
% b(3).FaceColor = [0 0.262 0.5];
% b(4).FaceColor = [0.3010 0.7450 0.9330];
% ylabel('Mean $\Delta p_e (m)$', 'Interpreter','latex')
% ax = gca;
% set(ax, 'FontSize', 18)
% set(ax, 'TickLabelInterpreter', 'latex')
% grid on
% box on
%
% bar_labels=categorical({'Reference (modeled)','Using controller (modeled)','Reference (measured)','Using controller (measured)'});
% bar_labels=reordercats(bar_labels,{'Reference (modeled)','Using controller (modeled)','Reference (measured)','Using controller (measured)'});
% hold on
% bh(1) = bar(nan,nan,'FaceColor',[0.6350 0.0780 0.1840]);
% bh(2) = bar(nan,nan,'FaceColor',[0.9290 0.6940 0.1250]);
% bh(3) = bar(nan,nan,'FaceColor',[0 0.262 0.5]);
% bh(4) = bar(nan,nan,'FaceColor',[0.3010 0.7450 0.9330]);
% legend(bh, string(bar_labels),'Location','eastoutside')
% l = findobj(gcf, 'Type', 'legend');
% set(l, 'Interpreter', 'latex')
% set(gcf, 'Position', [303 495 1115 331])
%
% %% Error mean plot with phi cost case
%
% load('..\data_in\0829-static_wkspaceCD\endpt_error_means.mat')
% ob_cats = categorical({'OB1','OB2','OB3','OB5'});
% ob_cats = reordercats(ob_cats,{'OB1','OB2','OB3','OB5'});
% b = bar(ob_cats,endpt_error_means);
% b(1).FaceColor = [0.6350 0.0780 0.1840];
% b(2).FaceColor = [0.8500 0.3250 0.0980];
% b(3).FaceColor = [0.9290 0.6940 0.1250];
% b(4).FaceColor = [0 0.262 0.5];
% % b(5).FaceColor = [0 0.4470 0.7410];
% % b(6).FaceColor = [0.3010 0.7450 0.9330];
% ylabel('Mean $\Delta p_e (m)$', 'Interpreter','latex')
% ax = gca;
% set(ax, 'FontSize', 18)
% set(ax, 'TickLabelInterpreter', 'latex')
% grid on
% box on
%
% bar_labels=categorical({'Reference (model)','No $\phi$ cost (model)','With $\phi$ cost (model)','Reference (meas.)','No $\phi$ cost (meas.)','With $\phi$ cost(meas.)'});
% bar_labels=reordercats(bar_labels,{'Reference (model)','No $\phi$ cost (model)','With $\phi$ cost (model)','Reference (meas.)','No $\phi$ cost (meas.)','With $\phi$ cost(meas.)'});
% hold on
% bh(1) = bar(nan,nan,'FaceColor',[0.6350 0.0780 0.1840]);
% bh(2) = bar(nan,nan,'FaceColor',[0.8500 0.3250 0.0980]);
% bh(3) = bar(nan,nan,'FaceColor',[0.9290 0.6940 0.1250]);
% bh(4) = bar(nan,nan,'FaceColor',[0 0.262 0.5]);
% bh(5) = bar(nan,nan,'FaceColor',[0 0.4470 0.7410]);
% bh(6) = bar(nan,nan,'FaceColor',[0.3010 0.7450 0.9330]);
% legend(bh, string(bar_labels),'Location','eastoutside')
% l = findobj(gcf, 'Type', 'legend');
% set(l, 'Interpreter', 'latex')
% set(gcf, 'Position', [303 495 1115 331])


%% Heatmap plots
topplt = 1; % Model comparison, control/nophicost/phicost, 1/2/3
btmplt = 4; % Measured comparison, control/nophicost/phicost, 4/5/6
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
set(ax, 'FontSize', 14)
set(ax, 'TickLabelInterpreter', 'latex')
grid on
box on

% Measured endpoints compared to goal points (nexttile 4,5,6)
nexttile(btmplt)
pcolor(xgrid_heatmap,zgrid_heatmap,[endpt_error zeros(nY,1); zeros(1,nX) 0])
caxis([0.0 0.2])
axis equal
xlim([-0.75,0.75]);
xticks(-0.6:0.2:0.6)
xticklabels('auto')
ylim([0 0.1*nY]);
xlabel('$x_B$ (m)','Interpreter','latex')
hold on
for i = 1:length(ts)
    plot([Endpt_Meas(1,i) Endpt_Sols(1,i)], [Endpt_Meas(2,i) Endpt_Sols(2,i)],':','color',plot_clr,'linewidth',1.5)
end
scatter(X_end_meas(:),Z_end_meas(:),20,plot_clr,'filled');
ax = gca;
set(ax, 'FontSize', 14)
set(ax, 'TickLabelInterpreter', 'latex')
grid on
box on

% clearvars -except t   % in between importing different datasets
%% Only ylabel on leftmost plots
nexttile(1)
ylabel('$y_B$ (m)','Interpreter','latex')
nexttile(4)
ylabel('$y_B$ (m)','Interpreter','latex')

%% Only colourbar on rightmost plots
nexttile(3)
ax = gca;
set(ax, 'FontSize', 14)
set(ax, 'TickLabelInterpreter', 'latex')
box on
cax = colorbar;
cax.Label.String = '$\Delta p_e$ (m)';
cax.Label.Interpreter = 'latex';
set(cax, 'TickLabelInterpreter', 'latex')
caxis([0.0 0.2])
nexttile(6)
ax = gca;
set(ax, 'FontSize', 14)
set(ax, 'TickLabelInterpreter', 'latex')
box on
cax = colorbar;
cax.Label.String = '$\Delta p_e$ (m)';
cax.Label.Interpreter = 'latex';
set(cax, 'TickLabelInterpreter', 'latex')
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

%% Orientation Experiment
% Note the names are all messed up, Goal_Phi is the goal for the tip angle (alpha), and Base_angle is the measured tip angle. Sad.

% Angle error plot
figure
scatter(-Goal_Phi(1:13),(Goal_Phi(1:13)-Base_angle(1:13)),60,[0 0.4470 0.7410],'filled') % Plot -ve Goal_Phi on x axis because in the report rotation opposite
xlabel('$\psi_{B^\prime}^*$ (rad)','Interpreter','latex')
xlim([-1.7,1.7])
xticks(-pi:pi/4:pi)%xticks(-180:45:180)
xticklabels({'$-\pi$','$-3\pi/4$','$-\pi/2$','$-\pi/4$','0','$\pi/4$','$\pi/2$','$3\pi/4$','$\pi$'})%xticklabels({'-180','-135','-90','-45','0','45','90','135','180'})
ylabel('$\Delta \psi_{e,B^\prime}$ (rad)','Interpreter','latex')
ylim([-0.15,0.3])
grid on
box on
ax = gca;
set(ax, 'FontSize', 36)
set(ax, 'TickLabelInterpreter', 'latex')

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

%% Plot an animated interpolation between solutions for defense presensation
q_int = interp1([0;1],[q_start; q_end],linspace(0,1,40)');
fig = figure;
fig.Position = [100 100 400 600];
for i = 1:length(q_int)
    q_i = q_int(i,:);
    scatter(-0.3,0.3,100,'kx','LineWidth',1.5)
    hold on 
    plot_config(q_i',1)
    box on
    drawnow
    ax = gca;
    exportgraphics(ax,'./frames/' + string(i) + '.png','Resolution',300)
end
