clear; close all



% the script that starts it all



% performance variables in the model
M_act_values = 0.1:0.1:2; % (1) mass of buoyancy engine actuator (ensure starts at zero)
d_values = 0.025:0.005:0.035; % (2) buoyancy engine diameter
l_values = 0.08:0.005:0.1; % (3) stroke of buoyancy engine
l_o_values = 30:20:70; % (4) neutral position of buoyancy engine
E_o_values = 0.3:0.05:0.4; % (5) location of moving mass

% d_values = d_values.^2;

% others parameters
M_hull = 1.1; % mass of glider when buoyancy engine fully empty (incl. payload) (yes)
D = 0.07; % diameter of glider hull (4 - 10 cm)
L = 0.8; %length of glider (0.87 - approx minimum length with 10cm stroke actuator, syringe and controllers in a small diameter hull - can make shorter in larger hull)

Y_act_ballast = -0.01;%-0.01; % y axis - dist away from centre line of the actuator and glider (which are coincident)
Y_hull_ballast = 0.01 - D/2; % y axis - dist away from glider centre line of the hull ballast

balance_hull_act_neutral = 50; % NOT a measurement position - where glider balances (%) with no ballast and actuator in neutral position
ballast_split = 0.01; % split of ballast between actuator (number as %) and the hull ballast (100 - number)


C_p_wing = [0; 0]; % wing at centre of glider should be somewhere betrween [0;0] and [cp;0]
Wing_length_total = L;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PERFORMANCE CRITERIA 
tmax = 200; % do calculations for tmax seconds of glide time
t_act = 33
% glider will start on surface, actuator will operate for t_act seconds and it will descend
% for tmax/2 seconds. actuator will then operate for t_act seconds and it will rise for
% tmax/2 seconds. model does not contain any environment other than water so
%  hitting the nominal surface (y=o) is meaningless (just getting a stable
%  glide ange out of it)

% have values here and pass them on
max_glide_angle = 35;% maximum downward and upward glide angle (in degrees) acceptable to be used 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
min_glide_angle_up = 10;
max_glide_angle_up = 30;
min_glide_angle_down = -70;
max_glide_angle_down = -90;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NEED MAX AND MIN GLIDE ANGLE BOTH UP AND DOWN - THEN CAN PLAY CAST AND TRAVEL PERFORMANCE CRITERIA
% THIS COULD BE THE MISSION PROFILE SECTION WHERE SAY WHAT WE WANT AND THE
% MODEL PRODUCES IT

min_dive_depth = 25; % if glider cannot achieve XX m depth in tmax/2 seconds then its sinking very slowly, not flying
% min or max velocity criteria
vel_y = 0.7; % abs y velocity component - more than 0.3 m/s its just falling or just floating

wobble_criteria = 5; % range in degrees of acceptable oscillation of glider on a stable heading

% time


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[up_coeffs, RMS_error_up, up_stats, down_coeffs, RMS_error_down, down_stats, M_glider, performance_glide_angle_up, performance_glide_angle_down, M_act_grid, d_grid, l_grid] = UW_glider_recipe_script(M_act_values, d_values, l_values, l_o_values, E_o_values, M_hull, D, L, Y_act_ballast, Y_hull_ballast, balance_hull_act_neutral, ballast_split, wobble_criteria, tmax, C_p_wing, Wing_length_total, max_glide_angle, min_dive_depth, vel_y, min_glide_angle_up, max_glide_angle_up, min_glide_angle_down, max_glide_angle_down, t_act);
disp('Upward Glide angle results')
up_coeffs
RMS_error_up
% up_stats
% prototype_up_glide_angle

disp('Downward Glide angle results')
down_coeffs
RMS_error_down
% down_stats
% prototype_down_glide_angle


%%
% x = M_act_values;
% y = d_values;
% z = l_values;
% V_down = performance_glide_angle_down(:,:,:,1,1)*180/pi;
% V_up = performance_glide_angle_up(:,:,:,1,1)*180/pi;
% 
% % [Xm, Ym, Zm] = meshgrid(x,y,z);
% figure(33)
% s = slice(y, x,z,V_down,[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% caxis([-75 -20]);
% ylabel(a,'Downward Glide Angle (Degrees)','FontSize',16)
% % set(gca,'XTicklabel',[])
% set(s,'EdgeColor','none');
% 
% figure(34)
% s = slice(y, x,z,-V_up,[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% caxis([-75 -20]);
% ylabel(a,'Upward Glide Angle (Degrees)','FontSize',16)
% % set(gca,'XTicklabel',[])
% set(s,'EdgeColor','none');
% 
% 
% figure(35)
% s = slice(y, x,z,(V_down-V_up)/2,[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% grid on
% grid minor
% caxis([-75 -20]);
% colormap(jet)
% ylabel(a,'Averaged Up and Down Glide Angle (Degrees)','FontSize',16)
% xlim([0.04 0.08]);
% ylim([1.5 5]);
% % zlim([0.05 0.2]);
% % set(gca,'XTicklabel',[])
% grayColor = [.7 .7 .7];
% set(s,'EdgeColor',grayColor,'FaceAlpha',1,'EdgeAlpha',0.1);
% 
% %% calculate the linear regression array and then the residuals between the regression and the model
% 
% % make array same size as performance array
% glide_angle_down_regress_5d = V_down;
% glide_angle_down_regress_5d(:) = 0;
% glide_angle_up_regress_5d = V_up;
% glide_angle_up_regress_5d(:) = 0;
% 
% for m = 1:length(y)%d_values
%     for n = 1:length(x) %M_act_values
%         for p = 1:length(z) % l_values
%             glide_angle_down_regress_5d(n, m, p) = (180/pi)*(down_coeffs(1) * x(n) + down_coeffs(2) * y(m)+ down_coeffs(3) * z(p) + down_coeffs(4) * mean(l_o_values) + down_coeffs(5) * mean(E_o_values) + down_coeffs(6));
%             glide_angle_up_regress_5d(n, m, p) = (180/pi)*(up_coeffs(1) * x(n) + up_coeffs(2) * y(m) + up_coeffs(3) * z(p) + up_coeffs(4) * mean(l_o_values) + up_coeffs(5) * mean(E_o_values) + up_coeffs(6));
%             glide_angle_down_regress_3d(n, m, p) = (180/pi)*(down_coeffs(1) * x(n) + down_coeffs(2) * y(m)+ down_coeffs(3) * z(p) + down_coeffs(6));
%             glide_angle_up_regress_3d(n, m, p) = (180/pi)*(up_coeffs(1) * x(n) + up_coeffs(2) * y(m) + up_coeffs(3) * z(p) + up_coeffs(6));
%             
%             
%         end
%     end
% end
% 
% residual_down_5d = abs(-glide_angle_down_regress_5d + V_down);
% residual_up_5d = abs(glide_angle_up_regress_5d - V_up);
% 
% residual_down_3d = abs(-glide_angle_down_regress_3d + V_down);
% residual_up_3d = abs(glide_angle_up_regress_3d - V_up);
% 
% 
% 
% figure(43)
% s = slice(y, x,z,residual_down_5d,[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% % caxis([0 5]);
% ylabel(a,'Downward Residuals (Degrees)','FontSize',16)
% % set(gca,'XTicklabel',[])
% set(s,'EdgeColor','none');
% 
% figure(44)
% s = slice(y, x,z,residual_up_5d,[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% % caxis([0 5]);
% ylabel(a,'Upward Residuals (Degrees)','FontSize',16)
% % set(gca,'XTicklabel',[])
% set(s,'EdgeColor','none');
% 
% figure(45)
% s = slice(y, x,z,(residual_down_5d + residual_up_5d)/2,[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% grid on
% grid minor
% caxis([-75 -20]);
% colormap(jet)
% ylabel(a,'Averaged Up and Down Glide Angle (Degrees)','FontSize',16)
% xlim([0.04 0.08]);
% ylim([1.5 5]);
% % zlim([0.05 0.2]);
% % set(gca,'XTicklabel',[])
% grayColor = [.7 .7 .7];
% set(s,'EdgeColor',grayColor,'FaceAlpha',1,'EdgeAlpha',0.1);
% 
% 
% 
% figure(53)
% s = slice(y, x,z,(residual_down_5d -residual_down_3d),[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% % caxis([0 5]);
% ylabel(a,'5d vs 3d Downward Residuals (Degrees)','FontSize',16)
% % set(gca,'XTicklabel',[])
% set(s,'EdgeColor','none');
% 
% figure(54)
% s = slice(y, x,z,(residual_up_5d -residual_up_3d),[d_values],[M_act_values],[l_values]);
% % slice(y, x,z,V1,[0.02 0.03],[M_act_values],[l_values])
% ylabel('Moving mass (kg)')
% xlabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% set(gca,'FontSize',18,'fontWeight','bold')
% a=colorbar;
% % caxis([0 5]);
% ylabel(a,'5d vs 3d Upward Residuals (Degrees)','FontSize',16)
% % set(gca,'XTicklabel',[])
% set(s,'EdgeColor','none');
% %%
% % % Mass parameter is the mass of and on the actuator / mass glider - M_act is almost an independent variable (volume parameters l_o and l are used to calculate hull-fitted ballast which impacts upon M_act).
% % % Volume metric is the volume of the buoyancy engine / volume of glider - Volume parameters d, l, and l_o are all interrelated.
% Vol_glider = pi * L* (D/2)^2;
% 
% Mass_metric = M_act_values/M_glider;
% Volume_metric = d_values.^2/Vol_glider;
% Engine_stroke_metric = l_values/L;
% Neutral_engine_length_metric = (l_o_values/100)/L;
% Ballast_location_metric = E_o_values/L;
% 
% resolution = 100;
% 
% M = 0:3/resolution:5;
% d = 0.02:(0.06 - 0.02)/resolution:0.08;
% 
% for n = 1:length(M)
%     
%     for m = 1:length(d)
%         % THINK THIS ALL THROUGH
%         l_up_25(m, n) = (25 * pi/180 - up_coeffs(1) * M(n) - up_coeffs(2) * d(m)- up_coeffs(6))/up_coeffs(3);
%         l_up_30(m, n) = (30 * pi/180 - up_coeffs(1) * M(n) - up_coeffs(2) * d(m)- up_coeffs(6))/up_coeffs(3);
%         l_up_35(m, n) = (35 * pi/180 - up_coeffs(1) * M(n) - up_coeffs(2) * d(m)- up_coeffs(6))/up_coeffs(3);
%         l_down_25(m, n) = -(-25 * pi/180 - up_coeffs(1) * M(n) - up_coeffs(2) * d(m)- up_coeffs(6))/up_coeffs(3);
%         l_down_30(m, n) = -(-30 * pi/180 - up_coeffs(1) * M(n) - up_coeffs(2) * d(m)- up_coeffs(6))/up_coeffs(3);
%         l_down_35(m, n) = -(-35 * pi/180 - up_coeffs(1) * M(n) - up_coeffs(2) * d(m)- up_coeffs(6))/up_coeffs(3);
% diameter(m,n) = d(m);
% Mass(m,n) = M(n);
%     end
% end
% 
% figure(1)
% % s = surf(Mass/M_glider, diameter/D, l_down_25/L,'FaceAlpha',0.1)
% % set(s, 'EdgeAlpha',0.1)
% surf(Mass, diameter, l_down_25, l_down_25*0+40)
% hold on
% surf(Mass, diameter, l_down_30, l_down_30*0+20)
% surf(Mass, diameter, l_down_35, l_down_25*0+30)
% 
% surf(Mass, diameter, l_up_25, l_up_25*0+40)
% surf(Mass, diameter, l_up_30, l_up_25*0+20)
% surf(Mass, diameter, l_up_35, l_up_25*0+30)
% % shading interp
% grid on
% % s_25 = [-45 0];
% % k_25 = [.65 .4 .3 10];
% % s_30 = [-30 0];
% % k_30 = [.45 .3 .2 8];
% % s_35 = [-15 0];
% % k_35 = [.25 .2 .1 6];
% 
% % surfl(Mass, diameter, l_down_25, s_25,k_25)
% % hold on
% % surfl(Mass, diameter, l_down_30, s_30,k_30)
% % surfl(Mass, diameter, l_down_35, s_35,k_35)
% % 
% % surfl(Mass, diameter, l_up_25, s_25,-k_25)
% % surfl(Mass, diameter, l_up_30, s_30,-k_30)
% % surfl(Mass, diameter, l_up_35, s_35,-k_35)
% % s_25.EdgeColor = 'none';
% % scatter3(Mass(:), diameter(:), (l_up_35(:)),'or')
% % mesh(M, d, l_down_25)
% % mesh(M, d, l_down_30)
% % mesh(M, d, l_down_35)
% % colorbar
% % caxis([0 0.2]);
% % plot3(0.60, 0.025, 0.1, 'o','Color','r','MarkerSize',10,'MarkerFaceColor','#D9FFFF') 
% hold off
% title('Design space. Glider Diameter 0.05m, Glider Length 1m')
% xlim([0 2]);
% ylim([0.02 0.05]);
% zlim([0.05 0.2]);
% xlabel('Moving mass (kg)')
% ylabel('Engine diameter (m)')
% zlabel('Engine stroke (m)')
% legend('\pm 25 glide angle','\pm 30 glide angle','\pm 35 glide angle')
% 
% 
% 
% %
% %
% % 
% % for aaa = 1:length(M_act_values) 4
% %        up_glide_angle_regression(aaa, bbb) = (M_act_values(aaa) * coeffs(1) + d_values(bbb) * coeffs(2) + l_values(2) * coeffs(3) + l_o_values(2) * coeffs(4) + E_o_values(2) * coeffs(5) + coeffs(6))*180/pi
% %     end
% % end
% % 
% % for bbb = 1:length(d_values)44
% % figure(9)
% % % contourf(M_act_values/M_glider, d_values/Vol_glider,up_glide_angle_regression)
% % colorbar
% % colormap(jet)
% % % Example of looking in one operating space
% % figure(10)
% % % contourf(M_act_grid(:,:,2,2,1)/M_glider, d_grid(:,:,2,2,1)/Vol_glider, performance_glide_angle_up(:,:,2,2,1)*180/pi)
% % imagesc(Volume_metric, Mass_metric, performance_glide_angle_up*180/pi,100, 'linestyle', 'none')
% % 
% % hold on
% % 
% % colorbar
% % colormap(jet)
% % caxis([10 60]);
% % xlim([0 20]);
% % ylim([0 40]);
% % axis xy
% % grid on
% % title('Upward Glide Angle')
% % xlabel('Engine volume/Glider volume (%)') % First Number in RATIO matrix
% % ylabel('Mass Actuator/Mass Glider (%)') % First Number in RATIO matrix
% % xlabel('Engine stroke length/Glider length (%)')
% % xlabel('Neutral engine position / Glider length')
% % %
% % figure(11)
% % imagesc(Ballast_location_metric, Mass_metric, performance_glide_angle_down*180/pi)
% % % contourf(Volume_metric, Mass_metric, performance_glide_angle_down*180/pi,100, 'linestyle', 'none')
% % colorbar
% % % caxis([-60 -10]);
% % % xlim([0 10]);
% % % ylim([0 40]);
% % colormap(flipud(jet))
% % axis xy
% % grid on
% % title('Downward Glide Angle')
% % xlabel('Engine volume/Glider volume (%)')
% % ylabel('Mass Actuator/Mass Glider (%)')
% % % xlabel('Engine stroke length/Glider length (%)')
% % % xlabel('Neutral engine position / Glider length')
% %
% % figure(12)
% % imagesc(Ballast_location_metric, Mass_metric, (-performance_glide_angle_down +  max(performance_glide_angle_down(:)) + performance_glide_angle_up - min(performance_glide_angle_up(:)))*180/(2*pi)+(-max(performance_glide_angle_down(:)) + min(performance_glide_angle_up(:)))*180/(2*pi))
% % % contourf(Volume_metric, Mass_metric, (-performance_glide_angle_down +  max(performance_glide_angle_down(:)) + performance_glide_angle_up - min(performance_glide_angle_up(:)))*180/(2*pi)+(-max(performance_glide_angle_down(:)) + min(performance_glide_angle_up(:)))*180/(2*pi),'ShowText','on')%,100, 'linestyle', 'none')
% % colorbar
% % % caxis([20 40]);
% % % xlim([0 4]);
% % % ylim([10 45]);
% % colormap(jet)
% % axis xy
% % grid on
% % title('Optimum Average Glide Angle Design Point ')
% % xlabel('Engine volume/Glider volume (%)')
% % ylabel('Mass Actuator/Mass Glider (%)')
% %
% %
% %%
% 
% 
% % l_layer = 10;
% % 
% % for mmmm = 1:length(M_act_values)
% %     
% %         dddd_up(mmmm) = (30 * pi/180 - l_values(10)*up_coeffs(3) - up_coeffs(6) - up_coeffs(1) * M_act_values(mmmm))/up_coeffs(2);
% % 
% %         dddd_down(mmmm) = (30 * pi/180 - l_values(10)*down_coeffs(3) - down_coeffs(6) - down_coeffs(1) * M_act_values(mmmm))/down_coeffs(2);
% %     
% %     
% %     
% % end
% % 
% % 
% % figure(2)
% % % s = surf(Mass/M_glider, diameter/D, l_down_25/L,'FaceAlpha',0.1)
% % % set(s, 'EdgeAlpha',0.1)
% % % surf(M_act_grid(:), d_grid(:), performance_glide_angle_down(:))
% % % imagesc(M_act_values/M_glider, d_values/D ,(performance_glide_angle_up(:,:,3)*180/pi  - performance_glide_angle_down(:,:,3)*180/pi) )
% % % imagesc(M_act_values/M_glider, d_values/D ,-performance_glide_angle_down(:,:,l_layer)*180/pi)
% % % imagesc(M_act_values/M_glider, d_values/D ,performance_glide_angle_up(:,:,3)*180/pi)
% % 
% % hold on
% % plot(M_act_values/M_glider, dddd_down,'x')
% % % imagesc(M_act_values, d_values,performance_glide_angle_down(:,:,2)*180/pi)
% % 
% % grid on
% % 
% % hold off
% % title('Design space (Up and Down 25, 30 and 30 degree slope), Glider Diameter 0.06m, Glider Length 1m ')
% % % xlim([0 3]);
% % % % ylim([0.2 1]);
% % % zlim([0 0.2]);
% % colorbar
% % colormap(flipud(jet))
% % caxis([20 75])
% % xlabel('Moving mass / Glider mass')
% % ylabel('Engine diameter / Glider diameter')
% % zlabel('Engine stroke / Glider length')
% % % legend('\pm 25 glide angle','\pm 30 glide angle','\pm 35 glide angle')
% % 
% % 
% % figure(3)
% % % s = surf(Mass/M_glider, diameter/D, l_down_25/L,'FaceAlpha',0.1)
% % % set(s, 'EdgeAlpha',0.1)
% % % surf(M_act_grid(:), d_grid(:), performance_glide_angle_down(:))
% % % imagesc(M_act_values/M_glider, d_values/D ,(performance_glide_angle_up(:,:,3)*180/pi  - performance_glide_angle_down(:,:,3)*180/pi) )
% % % imagesc(M_act_values/M_glider, d_values/D ,performance_glide_angle_down(:,:,3)*180/pi)
% % imagesc(M_act_values/M_glider, d_values/D ,performance_glide_angle_up(:,:,l_layer)*180/pi)
% % 
% % hold on
% % % imagesc(M_act_values, d_values,performance_glide_angle_down(:,:,2)*180/pi)
% % 
% % grid on
% % 
% % hold off
% % title('Design space (Up and Down 25, 30 and 30 degree slope), Glider Diameter 0.06m, Glider Length 1m ')
% % % xlim([0 3]);
% % % % ylim([0.2 1]);
% % % zlim([0 0.2]);
% % colorbar
% % colormap(flipud(jet))
% % caxis([20 75])
% % xlabel('Moving mass / Glider mass')
% % ylabel('Engine diameter / Glider diameter')
% % zlabel('Engine stroke / Glider length')
% % % legend('\pm 25 glide angle','\pm 30 glide angle','\pm 35 glide angle')
% % %%
% % 
% % 
% % %l_layer = 11;
% % 
% % figure(4)
% % % s = surf(Mass/M_glider, diameter/D, l_down_25/L,'FaceAlpha',0.1)
% % % set(s, 'EdgeAlpha',0.1)
% % % surf(M_act_grid(:), d_grid(:), performance_glide_angle_down(:))
% % % imagesc(M_act_values/M_glider, d_values/D ,(performance_glide_angle_up(:,:,3)*180/pi  - performance_glide_angle_down(:,:,3)*180/pi) )
% % % imagesc(M_act_values/M_glider, d_values/D ,performance_glide_angle_down(:,:,3)*180/pi)
% % imagesc(M_act_values/M_glider, d_values/D ,(performance_glide_angle_up(:,:,l_layer)*180/pi-performance_glide_angle_down(:,:,3)*180/pi)/2)
% % 
% % hold on
% % % imagesc(M_act_values, d_values,performance_glide_angle_down(:,:,2)*180/pi)
% % 
% % grid on
% % 
% % hold off
% % title('Design space - average up and down glide angle. Glider Diameter 0.08m, Glider Length 1m, Engine stroke    ')
% % % title('Design space - average up and down glide angle. Glider Diameter 0.06m, Glider Length 1m, Engine stroke 0.1m ')
% % % title('Design space - average up and down glide angle. Glider Diameter 0.06m, Glider Length 1m, Engine stroke 0.15m ')
% % % xlim([0 3]);
% % % % ylim([0.2 1]);
% % % zlim([0 0.2]);
% % colorbar
% % colormap(flipud(jet))
% % caxis([20 75])
% % xlabel('Moving mass / Glider mass')
% % ylabel('Engine diameter / Glider diameter')
% % zlabel('Engine stroke / Glider length')
% % % legend('\pm 25 glide angle','\pm 30 glide angle','\pm 35 glide angle')

%%

filename = 'glider_detail_11.mat';
save(filename)
