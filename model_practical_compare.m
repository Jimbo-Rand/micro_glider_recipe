% Run glider model script in mode 3 to get single dive profile 
% Ensure dimensions and times are same as prototype
% Figure 100 will show modelled compared to prototype flight
% Import experimental data
% Figure 101 will show angular prototype flight data


t_prac = 0:0.1:max(t);
pitch_prac_smooth = smoothdata(interp1(t,wrapToPi(theta)*180/pi, t_prac,'spline'));
pitch_prac = (interp1(t,wrapToPi(theta)*180/pi, t_prac,'spline'));

depth_prac = interp1(t,y, t_prac,'spline');


%%
% Load Data
[~, ~, raw] = xlsread('C:\Users\mjr46\OneDrive - University of Bath\Automation in action Paper 3\Modelling_glider_git\final_for_paper\llandegfedd_experiment_22_aug_24\processed_experiment_results_22_08_24_llandegfedd.xlsx','Processed_flight_6','B2:E2098');

raw(cellfun(@(x) ~isempty(x) && isnumeric(x) && isnan(x),raw)) = {''};
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),raw); % Find non-numeric cells
raw(R) = {NaN}; % Replace non-numeric cells
data = reshape([raw{:}],size(raw));

depth_exp = data(:,4);
time_depth_exp = data(:,3);
pitch_exp = data(:,2);
time_pitch_exp = data(:,1);

clearvars data raw R;



figure(100)
subplot(2, 1, 1)
yyaxis left
plot(t_prac,depth_prac, '-.','LineWidth',2.0)
% xlabel('time (s)')
set(gca,'XTicklabel',[])
% ylabel('Depth (m)')
ylim([-20 5])
%                                 xlim([0 max(t_prac)])
grid('on')
set(gca,'FontSize',18,'fontWeight','bold')

patch([12.5 -0.5 -0.5 12.5],[5 5 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
patch([120 95 95 120],[5 5 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
yyaxis right
plot(t_prac,pitch_prac, 'LineWidth',2.0)
%                                 hold on
%                                 plot(t_prac,pitch_prac_smooth, 'LineWidth',2.0)
% ylabel('Pitch (degrees)')
xlim([0 340])
ylim([-16 6])

subplot(2, 1, 2)
yyaxis left
plot(time_depth_exp,depth_exp,'-.', 'LineWidth',2.0)
xlabel('time (s)')
ylabel('                                      Depth (m)')
ylim([-20 5])
%                                 xlim([0 max(t_prac)])
grid('on')
set(gca,'FontSize',18,'fontWeight','bold')

yyaxis right

plot(time_pitch_exp,pitch_exp, 'LineWidth',2.0)
%                                 hold on
%                                 plot(t_prac,pitch_prac_smooth, 'LineWidth',2.0)
ylabel('                                 Pitch (degrees)')
xlim([0 340])
ylim([-16 6])
patch([12.5 -0.5 -0.5 12.5],[10 10 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)

patch([120 95 95 120],[10 10 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
legend('depth','pitch','buoyancy engine operating', 'location', 'best', 'NumColumns',1)

% Create textbox
annotation('textbox',...
    [0.158854166666667 0.150360453141092 0.0203125 0.0514933058702369],...
    'String','A',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.265625 0.128733264675592 0.0203125000000001 0.0514933058702369],...
    'String','B',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.489583333333334 0.361483007209063 0.0203125 0.0514933058702368],...
    'String','D',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.417187500000001 0.130792996910402 0.0203125 0.0514933058702368],...
    'String','C',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.690104166666668 0.294541709577755 0.0203125 0.0514933058702368],...
    'String','E',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.880208333333334 0.787847579814625 0.0203125 0.0514933058702367],...
    'String','F',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.277083333333334 0.796086508753862 0.0203125000000001 0.0514933058702367],...
    'String','G',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');





%%

figure(200)
subplot(2, 1, 1)

plot(t_prac,depth_prac, '-r','LineWidth',2.0)
hold on
plot(time_depth_exp,depth_exp,'-.k', 'LineWidth',2.0)
patch([12.5 -0.5 -0.5 12.5],[5 5 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
patch([120 95 95 120],[5 5 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
% xlabel('time (s)')
set(gca,'XTicklabel',[])
ylabel('Depth (m)')
ylim([-20 1])
xlim([0 340])
%                                 xlim([0 max(t_prac)])
grid('on')
set(gca,'FontSize',18,'fontWeight','bold')


subplot(2, 1, 2)


plot(t_prac,pitch_prac, '-r','LineWidth',2.0)
                                hold on

plot(time_pitch_exp,pitch_exp,'-.k', 'LineWidth',2.0)
%                                 hold on
%                                 plot(t_prac,pitch_prac_smooth, 'LineWidth',2.0)
ylabel('Pitch (degrees)')
xlim([0 340])
ylim([-16 6])
patch([12.5 -0.5 -0.5 12.5],[10 10 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)

patch([120 95 95 120],[10 10 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
set(gca,'FontSize',18,'fontWeight','bold')
legend('model','prototype','buoyancy engine operating', 'location', 'best', 'NumColumns',1)
xlabel('Time (s)')
grid on
% Create textbox
annotation('textbox',...
    [0.158854166666667 0.150360453141092 0.0203125 0.0514933058702369],...
    'String','A',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.265625 0.128733264675592 0.0203125000000001 0.0514933058702369],...
    'String','B',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.489583333333334 0.361483007209063 0.0203125 0.0514933058702368],...
    'String','D',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.417187500000001 0.130792996910402 0.0203125 0.0514933058702368],...
    'String','C',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.690104166666668 0.294541709577755 0.0203125 0.0514933058702368],...
    'String','E',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.880208333333334 0.787847579814625 0.0203125 0.0514933058702367],...
    'String','F',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

% Create textbox
annotation('textbox',...
    [0.277083333333334 0.796086508753862 0.0203125000000001 0.0514933058702367],...
    'String','G',...
    'FontWeight','bold',...
    'FontSize',14,...
    'FitBoxToText','off');

%%
% Load Data
[~, ~, raw] = xlsread('C:\Users\mjr46\OneDrive - University of Bath\Automation in action Paper 3\Modelling_glider_git\final_for_paper\llandegfedd_experiment_22_aug_24\processed_experiment_results_22_08_24_llandegfedd.xlsx','Processed_flight_9','B2:G1441');

raw(cellfun(@(x) ~isempty(x) && isnumeric(x) && isnan(x),raw)) = {''};
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),raw); % Find non-numeric cells
raw(R) = {NaN}; % Replace non-numeric cells
data = reshape([raw{:}],size(raw));

depth_exp = data(:,4);
time_depth_exp = data(:,3);
pitch_exp = data(:,2);
time_pitch_exp = data(:,1);
yaw_exp = data(:,5);
roll_exp = data(:,6);
clearvars data raw R;

%%

figure(101)
plot(time_pitch_exp,pitch_exp, 'k-','LineWidth',2.0)
hold on
plot(time_pitch_exp,yaw_exp, 'r-.','LineWidth',2.0)
plot(time_pitch_exp,roll_exp, 'b:','LineWidth',2.0)
patch([12.5 -0.5 -0.5 12.5],[5 5 -20 -20],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
xlabel('time (s)')
ylabel('Angle (degrees)')
ylim([-20 5])
grid('on')
set(gca,'FontSize',24,'fontWeight','bold')
xlim([0 90])
legend('pitch angle','yaw angle','roll angle','buoyancy engine operating', 'location', 'best', 'NumColumns',1)

%%

figure(102)
plot3(pitch_exp,time_pitch_exp,yaw_exp)
legend('pitch angle','yaw angle', 'location', 'best', 'NumColumns',2)
xlabel('time (s)')
ylabel('Yaw Angle (degrees)')
zlabel('Pitch Angle (degrees)')
grid on

