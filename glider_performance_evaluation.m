clear

% uses results from glider model - stored in Results sub-folder

internal_or_external = 1;
% 1 for external sweep
% 2 for internal sweep
%%

% if option 2 chosen above this selection needs to be made
d_or_l_sweep = 1;
% 1 for d sweep
% 2 for l sweep
% 3 for both
% NOTE: l sweep is not very enlightening, d is the key parameter

switch internal_or_external
    
    case 1
        % External parameter sweep
        L = [0.6:0.02:1.2]; % Hull length, m
        D = [0.03:0.002:0.1];% Hull diameter, m
        A = [0.04 0.08 0.12 0.16]; % Wing area, m^2
        
        L = [0.6:0.01:1.2]; % Hull length, m
        D = [0.03:0.001:0.1];% Hull diameter, m
        A = [0.04 0.08 0.12 0.16]; % Wing area, m^2
        
        
        
        % Fixed Internal values chosen in glider model
        l = 0.1;
        d = 0.03;
        m = 0.3;
        
        txt = 'ext';
        
    case 2
        
        % Internal parameter sweep
        
        
        
        switch d_or_l_sweep
            
            case 1
                % d sweep
                l = [0.05:0.01:0.15]; % Engine length, m
                d = [0.01:0.0002:0.05]; % Engine diameter, m
                m = [0.2 0.3 0.4]; % Moving ballast mass, kg
                
                l = [0.05:0.002:0.15]; % Engine length, m
                d = [0.01:0.0002:0.05]; % Engine diameter, m
                m = [0.2 0.3 0.4]; % Moving ballast mass, kg
                
            case 2
                % l sweep
                l = [0.05:0.0001:0.15];
                d = [0.01 0.02 0.03 0.04 0.05];
                m = [0.2 0.3 0.4];
                
            case 3
                % l and d sweep
                l = [0.05:0.01:0.15];
                d = [0.01:0.0002:0.05];
                m = [0.2 0.3 0.4];
                
        end
        
        % Internal parameter sweep
        L = 0.8;
        D = 0.05;
        A = 0.08;
        
     
        
        txt = 'int';
        
end

%% Loop over parameter combinations

counter = 1;

for i_L = 1:length(L)
    Li = L(i_L);
    for i_D = 1:length(D)
        Di = D(i_D);
        for i_A = 1:length(A)
            Ai = A(i_A);
            for i_l = 1:length(l)
                li = l(i_l);
                for i_d = 1:length(d)
                    di = d(i_d);
                    for i_m = 1:length(m)
                        mi = m(i_m);
                        
                        % Load file
                        filename = [txt '_L' num2str(Li) '_D' num2str(Di) '_A' num2str(Ai) '_l' num2str(li) '_d' num2str(di) '_m' num2str(mi) '.mat'];
                        load(['Results' filesep filename],'z','t','Li','Di','Ai','li','di','mi', 'trigger_1', 'M_total', 'C_m_down', 'C_m_up','C_b_down', 'C_m_up', 'xD', 'xL','g','rho')
                        
                        % % outputs from solving the equations of motion
                        x = z(:,1); % x-position - horizontal distance travelled
                        vx = z(:,2); % x-velocity - horizontal velocity
                        y = z(:,3); % y-position - depth
                        vy = z(:,4); % y-velocity - vertical velocity
                        theta = z(:,5); % pitch angle
                        vtheta = z(:,6); % pitch angular velocity
                        
                        % derived outputs
                        phi = atan2(vy, vx); % glide angle
                        attack = wrapToPi(theta - phi); % angle of attack
                        
                        %% Evaluate performance
                        
                        % Assume up and down are the same
                        eval_fraction = 0.1; % change eval fraction value to different value (ie 0.1 to capture last 10% of down flight and 10% of up flight)
                        
                        % Find turning point
                        Iturn = find( abs(t-trigger_1) == min(abs(t-trigger_1)) );
                        
                        % Find the evalaution regions
                        t1 = trigger_1-trigger_1*eval_fraction;
                        I1 = find( abs(t-t1) == min(abs(t-t1)) );
                        
                        % down glide
                        xdown = x(I1:Iturn);
                        ydown = y(I1:Iturn);
                        thetadown = theta(I1:Iturn);
                        phidown = phi(I1:Iturn);
                        
                        t2 = trigger_1+trigger_1*(1-eval_fraction);
                        I2 = find( abs(t-t2) == min(abs(t-t2)) );
                        
                        % up glide
                        xup = x(I2:end);
                        yup = y(I2:end);
                        thetaup = theta(I2:end);
                        phiup = phi(I2:end);
                        
                        
                        % glide angle of individual legs over the eval_fraction of that flights leg
                        glide_angle_down = mean( atan(vy(I1:Iturn)./vx(I1:Iturn)) );
                        glide_angle_up = mean( atan(vy(I2:end)./vx(I2:end)) );
                        
                        pitch_angle_down = mean(thetadown);
                        pitch_angle_up = mean(thetaup);
                        
                        % average standard deviation on eval section of paths
                        pitch_wobble_down = std(thetadown);
                        pitch_wobble_up = std(thetaup);
                        
                        
                        % remove any impossible results
                        if Di < di
                            phiup = NaN;
                            glide_angle_up = NaN;
                            thetaup = NaN;
                            phidown = NaN;
                            glide_angle_down = NaN;
                            thetadown = NaN;
                        end
                        
                        % collect the results over the full range of design variables
                        up_glide_angle(i_L,i_D,i_A,i_l,i_d,i_m) =  glide_angle_up;
                        down_glide_angle(i_L,i_D,i_A,i_l,i_d,i_m) =  glide_angle_down;
                        up_pitch_angle(i_L,i_D,i_A,i_l,i_d,i_m) = pitch_angle_up;
                        down_pitch_angle(i_L,i_D,i_A,i_l,i_d,i_m) = pitch_angle_down;
                        up_pitch_wobble(i_L,i_D,i_A,i_l,i_d,i_m) = pitch_wobble_up;
                        down_pitch_wobble(i_L,i_D,i_A,i_l,i_d,i_m) = pitch_wobble_down;
                        
                        
                        % obtain mean lift and drag parameters over eval fraction
                        attack_down = mean( attack(I1:Iturn) );
                        attack_up = mean( attack(I2:end) );
                        
                        Vel_down = sqrt( mean(vy(I1:Iturn) )^2 + ( mean( vx(I1:Iturn) ) )^2 );
                        Vel_up = sqrt( ( mean(vy(I2:end) ) )^2 + ( mean( vx(I2:end) ) )^2 );
                        
                        [Cd_down, Cl_down] = liftdrag(attack_down, Vel_down); % Cl_down for mean angle of attack down
                        [Cd_up, Cl_up] = liftdrag(attack_up, Vel_up); % Cl_up for mean angle of attack up
                        
                        pitch_down = mean( theta(I1:Iturn) );
                        pitch_up = mean( theta(I2:end) );
                        
                        mean_attack_down = attack_down *180/pi;
                        mean_attack_up = attack_up *180/pi;
                        
                        shadow_area_down = abs( ( pi * ( di/2 )^2 + ( 0.1 * Ai ) ) * cos(glide_angle_down) + ( ( Li * Di ) + (Ai) ) * sin(glide_angle_down) );
                        shadow_area_up = abs( ( pi * ( di/2 )^2 + ( 0.1 * Ai ) ) * cos(glide_angle_up) + ( ( Li * Di ) + (Ai) ) * sin(glide_angle_up) );
                        
                        F_Lift_down_x = 0.5 * rho * Cl_down * shadow_area_down *( ( mean( vx(I1:Iturn) ) )^2 + ( mean( vy(I1:Iturn) ) )^2  ) * cos (glide_angle_down);
                        F_Lift_up_x = 0.5 * rho * Cl_up * shadow_area_up * ( ( mean( vx(I2:end) ) )^2 + ( mean( vy(I2:end) ) )^2  ) * cos (glide_angle_up);
                        
                        F_Drag_down_x = 0.5 * rho * Cd_down * shadow_area_down * ( ( mean( vx(I1:Iturn) ) )^2 + ( mean( vy(I1:Iturn) ) )^2  ) * sin (glide_angle_down);
                        F_Drag_up_x = 0.5 * rho * Cd_up * shadow_area_up * ( ( mean( vx(I2:end) ) )^2 + ( mean( vy(I2:end) ) )^2  ) * sin (glide_angle_up);
                        
                        B_glider_down = rho * g * pi * ( Li * ( Di/2 )^2 - li * ( di/2 )^2 );
                        B_glider_up = rho * g * pi * Li * ( Di/2 )^2;
                        
                        W_glider = g * M_total;
                        
                        down_buoyancy(i_L,i_D,i_A,i_l,i_d,i_m) = B_glider_down;
                        up_buoyancy(i_L,i_D,i_A,i_l,i_d,i_m) = B_glider_up;
                        weight(i_L,i_D,i_A,i_l,i_d,i_m) = W_glider;
                        
                        
                        % Centre of Equilibrium (true centre of rotation during stable flight)
                        
                        % CoE_down_dynamic(i_L,i_D,i_A,i_l,i_d,i_m) = -( W_glider * C_m_down(1) - B_glider_down * C_b_down(1) - abs(F_Lift_down_x)  * -xL(1)) +  abs(F_Drag_down_x)  * -xD(1)) / ( W_glider - B_glider_down - abs(F_Lift_down_x) - abs(F_Drag_down_x) );%( W_glider * C_m_down(1) - B_glider_down * C_b_down(1) + F_Drag_down_x * xD(1) ) / ( W_glider - B_glider_down - F_Lift_down_x - F_Drag_down_x );
                        % CoE_up_dynamic(i_L,i_D,i_A,i_l,i_d,i_m) = -( W_glider * -C_m_up(1) -  abs(F_Lift_up_x)  * -xL(1) -  abs(F_Drag_up_x)  * -xD(1)) / ( W_glider - B_glider_up - abs(F_Lift_down_x) + abs(F_Drag_down_x) );%-( W_glider * C_m_up(1) + F_Drag_up_x * xD(1) ) / ( W_glider - B_glider_up - F_Lift_up_x - F_Drag_up_x );
                        
                        CoE_down_static(i_L,i_D,i_A,i_l,i_d,i_m) = ( ( abs( W_glider ) * C_m_down(1)  - abs( B_glider_down * C_b_down(1) ) ) / ( abs(W_glider) - abs(B_glider_down) ) );
                        CoE_up_static(i_L,i_D,i_A,i_l,i_d,i_m) = ( ( abs( W_glider ) * -C_m_up(1)  ) / ( abs( W_glider) - abs(B_glider_up) ) );
                        
                        %                         CoE_down_static =  ( abs( M_total * g ) .* l_w_down_x  - abs( L*pi*(D/2)^2 - l*pi*(d/2)^2)*rho*g .* l_b_down  )  /( ( abs( M_total * g ) - abs( L*pi*(D/2)^2 - l*pi*(d/2)^2)*rho*g ) );
                        %                         CoE_up_static = ( abs( M_total * g ) .* -(l_w_up_x  )) / ( abs( M_total * g) - abs( (L*pi*(D/2)^2)*rho*g ) );
                        
                        
                        
                        
                        vel_mean_down(i_L,i_D,i_A,i_l,i_d,i_m) = Vel_down;
                        vel_mean_up(i_L,i_D,i_A,i_l,i_d,i_m) = Vel_up;
                        
                        l_m_down(i_L,i_D,i_A,i_l,i_d,i_m) = C_m_down(1);
                        l_m_up(i_L,i_D,i_A,i_l,i_d,i_m) = C_m_up(1);
                        l_b_down(i_L,i_D,i_A,i_l,i_d,i_m) = C_b_down(1);
                        
                        up_attack(i_L,i_D,i_A,i_l,i_d,i_m) = mean_attack_up;
                        down_attack(i_L,i_D,i_A,i_l,i_d,i_m) = mean_attack_down;
                        
                        counter = counter + 1;
                        Percent_complete = 100*counter/( length(D) * length(L) * length(A) * length(d) * length(l) * length(m) )
                        
                    end
                end
            end
        end
    end
end


%% Examine data points

switch internal_or_external
    case 1% external variables plotted from here
        
        where_external_points = 2; % used to put 4 fixed data points on the performance graphs and plot each points full flight for comparison
        % 1 = LH bottom
        % 2 = LH top
        % 3 = RH top
        % 4 = RH bottom
        
        % 5 = straight down
        % 6 = straight across
        % 7 = 4 regions
        % 8 = Across min glide angle hump
        % 9 = just dropping off the ridge (first point) away from coe
        % 10 = in sweet spot but straying over coe
        % 11 = boxing the sweet spot for A = 0.02 m^2
        % 12 = across hump towards coe
        % 13 = over hump, away from coe
        % 14 = same as 13 but lower L, higher D
        
        switch where_external_points
            
            case 1
                L0 = [0.8 0.98 1.14 1.14];
                D0 = [0.064 0.098 0.09 0.043];
                
            case 2
                L0 = [0.7 0.75 0.8 0.9];
                D0 = [0.045 0.04 0.035 0.03];
                
            case 3
                L0 = [1.02 1.02 1.02 1.02];
                D0 = [0.078 0.08 0.082 0.084];
                
            case 4
                L0 = [0.96 0.98 1.0 1.02];
                D0 = [0.09 0.09 0.09 0.09];
                
            case 5
                L0 = [0.96 1.0 1.04 1.08];
                D0 = [0.04 0.04 0.04 0.04];
                
            case 6
                L0 = [0 0 0 0];
                D0 = [0 0 0 0 ];
                
            case 7
                L0 = [0.7 0.74 0.82 0.86];
                D0 = [0.05 0.06 0.078 0.082];
                
            case 8
                L0 = [0.78 0.84 0.92 0.98];
                D0 = [0.076 0.068 0.052 0.04];
                
            case 9
                L0 = [0.66 0.74 0.82 0.96];
                D0 = [0.092 0.078 0.064 0.036];
                
            case 10
                L0 = [0.82 0.84 0.88 0.92];
                D0 = [0.064 0.058 0.054 0.044];
                
            case 11
                L0 = [0.82 0.82 0.64 0.64];
                D0 = [0.054 0.062 0.074 0.088];
                
            case 12
                L0 = [0.82 0.84 0.86 0.88];
                D0 = [0.064 0.068 0.072 0.076];
                
            case 13
                L0 = [0.76 0.78 0.8 0.82];
                D0 = [0.05 0.054 0.058 0.062];
                
            case 14
                L0 = [0.64 0.68 0.72 0.76];
                D0 = [0.058 0.062 0.068 0.072];
        end
        
        i_A = 1;
        A0 = A(i_A);
        
        
        figure(1)
        down_glide_angle(down_glide_angle>60*pi/180)=NaN; % clean up high angles so figure looks clearner
        imagesc(L,D,squeeze(down_glide_angle(:,:,i_A)).'*180/pi)
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Glide angle, degrees';
        caxis([-60 0])
        %         title(['Down glide angle: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold')
        % Create textbox
        annotation('textbox',...
            [0.162458333333333 0.168898043254377 0.0716145833333333 0.0525231719876416],...
            'String',{'DIVING'},...
            'FontWeight','bold',...
            'FontSize',14,...
            'BackgroundColor',[1 1 1]);
        
        % Create textbox
        annotation('textbox',...
            [0.762979166666667 0.133882595262616 0.0877604166666667 0.0525231719876416],...
            'String',{'FLUTTER'},...
            'FontWeight','bold',...
            'FontSize',14,...
            'BackgroundColor',[1 1 1]);
        
        % Create textbox
        annotation('textbox',...
            [0.720791666666668 0.535530381050464 0.116187499999999 0.0834191555097838],...
            'String',{'PROGRESSIVE','    FLUTTER'},...
            'FontWeight','bold',...
            'FontSize',14,...
            'FitBoxToText','off',...
            'BackgroundColor',[1 1 1]);
        
        % Create textbox
        annotation('textbox',...
            [0.440062500000001 0.782698249227601 0.10078125 0.0525231719876416],...
            'String',{'BOUNDING'},...
            'FontWeight','bold',...
            'FontSize',14,...
            'BackgroundColor',[1 1 1]);
        
        % Create arrow
        annotation('arrow',[0.534895833333333 0.6078125],...
            [0.824922760041195 0.874356333676622],'Color',[1 1 1],'LineWidth',5);
        
%         % Create line
%         annotation('line',[0.565625 0.563541666666667],...
%             [0.110225540679712 0.920700308959835],'Color',[1 0 0],'LineWidth',5,...
%             'LineStyle','-.');
        
        % Create textbox
        annotation('textbox',...
            [0.1744375 0.58908341915551 0.08203125 0.0525231719876416],...
            'String',{'GLIDING'},...
            'FontWeight','bold',...
            'FontSize',14,...
            'BackgroundColor',[1 1 1]);
        
        
        
        figure(2)
        imagesc(L,D,squeeze(down_pitch_angle(:,:,i_A)).'*180/pi)
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch angle, degrees';
        caxis([-40 0])
        %         title(['Down pitch angle: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold')
        
        figure(3)
        imagesc(L,D,squeeze(down_pitch_wobble(:,:,i_A)).'*180/pi)
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch wobble, degrees';
        caxis([0 6])
        %         title(['Down pitch wobble: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold')
        
        figure(4)
        imagesc(L,D,squeeze(CoE_down_static(:,:,i_A)).')
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'CoE position relative to C_G, m';
        caxis([-0.1 0.1])
        title(['COE down: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',18,'fontWeight','bold')
        
        figure(5)
        imagesc(L,D,squeeze(up_glide_angle(:,:,i_A)).'*180/pi)
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Glide angle, degrees';
        caxis([0 60])
        title(['Up glide angle: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',18,'fontWeight','bold')
        
        figure(6)
        imagesc(L,D,squeeze(up_pitch_angle(:,:,i_A)).'*180/pi)
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch angle, degrees';
        caxis([0 20])
        title(['Up pitch angle: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',18,'fontWeight','bold')
        
        figure(7)
        imagesc(L,D,squeeze(up_pitch_wobble(:,:,i_A)).'*180/pi)
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch wobble, degrees';
        caxis([0 10])
        title(['Up pitch wobble: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',18,'fontWeight','bold')
        
        figure(8)
        imagesc(L,D,squeeze(CoE_up_static(:,:,i_A)).')
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'CoE, m';
        caxis([-0.1 0.1])
        title(['COE up: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',18,'fontWeight','bold')
        
        figure(9)
        imagesc(L,D,squeeze(vel_mean_down(:,:,i_A)).')
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Velocity, ms^{-1}';
        caxis([0.6 1.4])
        title(['Down mean velocity: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold')
        
 %%       
        
        figure(900)
        imagesc(L,D,squeeze(vel_mean_down(:,:,1) - vel_mean_down(:,:,4)).')
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Velocity difference, ms^{-1}';
        caxis([0.0 0.2])
        title(['Down mean velocity: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold')
        

%%        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        figure(10)
        imagesc(L,D,squeeze(vel_mean_up(:,:,i_A)).')
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Velocity, ms^{-1}';
        caxis([0.6 1.4])
        title(['Velocity mean up: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',18,'fontWeight','bold')
        
  %%      
               figure(910910)
        imagesc(L,D,squeeze(vel_mean_down(:,:,i_A) - vel_mean_up(:,:,i_A)).')
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Velocity difference, ms^{-1}';
        caxis([0 0.3])
%         title(['Down mean velocity: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold') 
        
        
 %%       
        
        
        figure(10000)
        up_down_diff = squeeze(down_glide_angle(:,:,i_A)).'*180/pi + squeeze(up_glide_angle(:,:,i_A)).'*180/pi;
        up_down_diff(up_down_diff>0)=NaN; % clean up high angles so figure looks clearner
        up_down_diff(up_down_diff<-10)=NaN; % clean up high angles so figure looks clearner
        %         down_glide_angle(down_glide_angle>60*pi/180)=NaN; % clean up high angles so figure looks clearner
        imagesc(L,D,(up_down_diff))
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Glide angle, degrees';
        caxis([-10 0])
        %         title(['Down glide angle: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold')
        
        figure(10001)
        up_down_buoy_diff = squeeze(down_buoyancy(:,:,i_A));
        imagesc(L,D,(up_down_buoy_diff))
        axis xy
        xlabel('Hull Length L, m')
        ylabel('Hull Diameter D, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Buoyancy Difference, m^3';
        caxis([20 40])
        %         title(['Down glide angle: A = ' num2str(A(i_A)) 'm^2'])
        set(gca,'FontSize',22,'fontWeight','bold')
        
        
        
        symbols = {'ko','k^','ks','kd'};
        symbol_shape = {'circle','triangle','square','diamond'};
        
        for n=1:length(L0)
            
            Ln = L0(n);
            Dn = D0(n);
            
            for p=1:10
                figure(p)
                hold on
                plot(Ln,Dn,symbols{n},'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',14)
                hold off
            end
            
            % Load file
            filename = [txt '_L' num2str(Ln) '_D' num2str(Dn) '_A' num2str(A0) '_l' num2str(l) '_d' num2str(d) '_m' num2str(m) '.mat'];
            load(['Results' filesep filename])
            
            % % outputs from solver
            x = z(:,1); % x-position - horizontal distance travelled
            vx = z(:,2); % x-velocity - horizontal velocity
            y = z(:,3); % y-position - depth
            vy = z(:,4); % y-velocity - vertical velocity
            theta = z(:,5); % pitch angle
            vtheta = z(:,6); % pitch angular velocity
            
            % derived outputs
            phi = atan2(vy, vx); % glide angle
            attack = wrapToPi(theta - phi); % angle of attack
            
          
            
            figure(10+n)
            
            subplot(2,1,1)
            yyaxis left
            plot(x,y, 'LineWidth',2.0)
            %     axis equal
            xlim([0 max(x)])
            ylim([min(y)-5 0])
            ylabel('Depth m')
            xlabel('Distance m')
            set(gca,'FontSize',10)
            grid on
            yyaxis right
            plot(x,sqrt(vy.^2 + vx.^2), 'LineWidth',2.0)
            hold on
            %plot(max(x)/2,1.2,symbols{n},'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',14)
            ylim([0 1.2])
          
            ylabel('Velocity m/s')
            set(gca,'FontSize',22,'fontWeight','bold')
            
            grid on
            
            subplot(2,1,2)
            plot(t,wrapTo180(theta*180/pi), 'LineWidth',2.0)
            hold on
            plot(t,wrapTo180(attack*180/pi), 'LineWidth',2.0)
            plot(t,wrapTo180(phi*180/pi), 'LineWidth',2.0)
            plot(t([1 end]),[1 1]*90,'k--', 'LineWidth',1.0)
            plot(t([1 end]),[1 1]*-90,'k--', 'LineWidth',1.0)
%             plot(max(t)/2,100,symbols{n},'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',28)
            plot(0,100,symbols{n},'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',28)            
            colormap(flipud(parula))
            ylabel('Angle degrees')
            xlabel('time s')
            hold off
            legend({'Pitch','Attack','Glide'},'Location','Best','NumColumns',3)
            %title( [ symbols{n}],'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',14)
            ylim([-100 100])
            set(gca,'YTick',[-90 -45 0 45 90])
            xlim([0 300])
            set(gca,'FontSize',22,'fontWeight','bold')
            grid on
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
        end
        
    case 2
        
        where_internal_points = 1; % used to put 4 fixed data points on the performance graphs and plot each points full flight for comparison
        % 1 = 0.2kg points
        % 2 = 0.4 kg points
        % 3 = l= 0.05m
        % 4 = RH bottom
        % 5 = straight across
        % 6 = straight down
        % 7 = sweet spot m = 0.4kg note diamond on a std syringe size
        
        switch where_internal_points
            
            case 1
                l0 = [0.05 0.06 0.07 0.08];
                d0 = [0.0388 0.0402 0.0416 0.0436];
                
            case 2 % m=0.4
                l0 = [0.07 0.09 0.11 0.13];
                d0 = [0.03 0.036 0.0394 0.041];
                
            case 3% for m = 0.2
                l0 = [0.13 0.068 0.092 0.06 ];
                d0 = [0.0252 0.0274 0.0302 0.0342 ];
                
            case 4
                l0 = [0 0 0 0];
                d0 = [0 0 0 0]; % l=0.1, m = 0.2
                
            case 5
                l0 = [0.1 0.1 0.1 0.1];
                d0 = [0.0193 0.0189 0.0188 0.019];
                
            case 6
                l0 = [0.1 0.1 0.1 0.1];
                d0 = [0.01 0.02 0.03 0.038];
                
            case 7
                l0 = [0.12 0.13 0.14 0.15];
                d0 = [0.028 0.028 0.028 0.028];
                
        end
        %
        i_m = 1;
        m0 = m(i_m);
        internal_down_glide_clean = down_glide_angle;
        internal_down_glide_clean(internal_down_glide_clean>85*pi/180)=NaN; % clean up high angles so figure looks clearner
        
        figure(21)
        imagesc(l,d,squeeze(internal_down_glide_clean(i_L,i_D,i_A,:,:,i_m)).'*180/pi)
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Glide angle, degrees';
        set(gca,'FontSize',22,'fontWeight','bold')
        caxis([-40 0])
        %         title(['Down glide angle: m = ' num2str(m(i_m)) 'kg'])
        % Create textbox
        
            annotation('textbox',...
            [0.575479166666667 0.175077239958805 0.0716145833333334 0.0525231719876416],...
            'String','DIVING',...
            'FontWeight','bold',...
            'FontSize',14,...
            'EdgeColor',[1 1 1],...
            'BackgroundColor',[1 1 1]);
        
        % Create textbox
        annotation('textbox',...
            [0.243708333333333 0.29351184346035 0.08203125 0.0525231719876416],...
            'String','GLIDING',...
            'FontWeight','bold',...
            'FontSize',14,...
            'EdgeColor',[1 1 1],...
            'BackgroundColor',[1 1 1]);
        
        % Create textbox
        annotation('textbox',...
            [0.124958333333334 0.494335736354274 0.198177083333333 0.0525231719876416],...
            'String','PROGRESSIVE FLUTTER',...
            'FontWeight','bold',...
            'FontSize',14,...
            'EdgeColor',[1 1 1],...
            'BackgroundColor',[1 1 1]);
        
        % Create textbox
        annotation('textbox',...
            [0.498395833333334 0.70442842430484 0.0877604166666667 0.0525231719876416],...
            'String','FLUTTER',...
            'FontWeight','bold',...
            'FontSize',14,...
            'EdgeColor',[1 1 1],...
            'BackgroundColor',[1 1 1]);
        
        % Create textbox
        annotation('textbox',...
            [0.651520833333338 0.572605561277031 0.10078125 0.0525231719876416],...
            'String','BOUNDING',...
            'FontWeight','bold',...
            'FontSize',14,...
            'EdgeColor',[1 1 1],...
            'BackgroundColor',[1 1 1]);
        
        % Create arrow
        annotation('arrow',[0.661458333333334 0.622395833333334],...
            [0.574695159629248 0.513903192584963],'Color',[1 1 1],'LineWidth',5);
        

        
        figure(2121)
        imagesc(l,d,squeeze(CoE_down_static(i_L,i_D,i_A,:,:,i_m)).')
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'CoE';
        set(gca,'FontSize',18,'fontWeight','bold')
        %         caxis([-60 0])
        title(['CoE Down: m = ' num2str(m(i_m)) 'kg'])
        
        
        
        figure(22)
        imagesc(l,d,squeeze(down_pitch_angle(i_L,i_D,i_A,:,:,i_m)).'*180/pi)
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch angle, degrees';
        set(gca,'FontSize',22,'fontWeight','bold')
        caxis([-20 0])
        %         title(['Down pitch angle: m = ' num2str(m(i_m)) 'kg'])
        
        
        
        figure(23)
        imagesc(l,d,squeeze(down_pitch_wobble(i_L,i_D,i_A,:,:,i_m)).'*180/pi)
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch wobble, degrees';
        set(gca,'FontSize',22,'fontWeight','bold')
        caxis([0 3])
        %         title(['Down pitch wobble: m = ' num2str(m(i_m)) 'kg'])
        
        figure(24)
        imagesc(l,d,squeeze(CoE_down_static(i_L,i_D,i_A,:,:,i_m)).')
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'CoE, m';
        set(gca,'FontSize',18,'fontWeight','bold')
        caxis([0 0.5])
        title(['COE down: m = ' num2str(m(i_m)) 'm'])
        
        figure(25)
        imagesc(l,d,squeeze(up_glide_angle(i_L,i_D,i_A,:,:,i_m)).'*180/pi)
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Glide angle, degrees';
        set(gca,'FontSize',18,'fontWeight','bold')
        caxis([0 40])
        title(['Up glide angle: m = ' num2str(m(i_m)) 'kg'])
        
        figure(26)
        imagesc(l,d,squeeze(up_pitch_angle(i_L,i_D,i_A,:,:,i_m)).'*180/pi)
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch angle, degrees';
        set(gca,'FontSize',18,'fontWeight','bold')
        caxis([0 20])
        title(['Up pitch angle: m = ' num2str(m(i_m)) 'kg'])
        
        figure(27)
        imagesc(l,d,squeeze(up_pitch_wobble(i_L,i_D,i_A,:,:,i_m)).'*180/pi)
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Pitch wobble, degrees';
        set(gca,'FontSize',18,'fontWeight','bold')
        caxis([0 3])
        title(['Up pitch wobble: m = ' num2str(m(i_m)) 'kg'])
        
        figure(28)
        imagesc(l,d,squeeze(CoE_up_static(i_L,i_D,i_A,:,:,i_m)).')
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'CoE, m';
        set(gca,'FontSize',18,'fontWeight','bold')
        caxis([-0.5 0.1])
        title(['COE up: m = ' num2str(m(i_m)) 'm'])
        
        figure(29)
        imagesc(l,d,squeeze(vel_mean_down(i_L,i_D,i_A,:,:,i_m)).')
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Velocity, ms^{-1}';
        set(gca,'FontSize',18,'fontWeight','bold')
        title(['Down mean velocity: m = ' num2str(m(i_m)) 'kg'])
        
        figure(30)
        imagesc(l,d,squeeze(vel_mean_up(i_L,i_D,i_A,:,:,i_m)).')
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Velocity, ms^{-1}';
        set(gca,'FontSize',18,'fontWeight','bold')
        title(['Up mean velocity: m = ' num2str(m(i_m)) 'kg'])
        
        
        figure(210000)
        up_down_diff = squeeze(down_glide_angle(i_L,i_D,i_A,:,:,i_m)).'*180/pi + squeeze(up_glide_angle(i_L,i_D,i_A,:,:,i_m)).'*180/pi;
        up_down_diff(up_down_diff>0)=NaN; % clean up high angles so figure looks clearner
        up_down_diff(up_down_diff<-10)=NaN; % clean up high angles so figure looks clearner
        imagesc(l,d,(up_down_diff))
        axis xy
        xlabel('l, m')
        ylabel('d, m')
        colormap(parula)
        c = colorbar;
        c.Label.String = 'Angle difference, degrees';
        set(gca,'FontSize',22,'fontWeight','bold')
        caxis([-10 0])
        %title(['up and down difference glide angle: m = ' num2str(m(i_m)) 'kg'])
        
        
        
        
        
        
        
        
        
        
        symbols = {'ko','k^','ks','kd'};
        symbol_shape = {'circle','triangle','square','diamond'};
        
        for n=1:length(l0)
            
            ln = l0(n);
            dn = d0(n);
            
            for p=1:10
                figure(p+20)
                hold on
                plot(ln,dn,symbols{n},'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',14)
                hold off
            end
            
            % Load file
            filename = [txt '_L' num2str(L) '_D' num2str(D) '_A' num2str(A) '_l' num2str(ln) '_d' num2str(dn) '_m' num2str(m0) '.mat'];
            load(['Results' filesep filename])
            
            
            % % outputs from solver
            x = z(:,1); % x-position - horizontal distance travelled
            vx = z(:,2); % x-velocity - horizontal velocity
            y = z(:,3); % y-position - depth
            vy = z(:,4); % y-velocity - vertical velocity
            theta = z(:,5); % pitch angle
            vtheta = z(:,6); % pitch angular velocity
            
            % derived outputs
            phi = atan2(vy, vx); % angle of flight
            attack = wrapToPi(theta - phi); % angle of attack
            
            
            figure(30+n)
            
            subplot(2,1,1)
            yyaxis left
            plot(x,y, 'LineWidth',2.0)
            %     axis equal
            %            xlim([min(x) 0])
            %   xlim([0 max(x)])
            ylim([min(y)-5 0])
            ylabel('Depth m')
            xlabel('Distance m')
            set(gca,'FontSize',10)
            grid on
            yyaxis right
            plot(x,sqrt(vy.^2 + vx.^2), 'LineWidth',2.0)
            hold on
            %plot(max(x)/2,1.2,symbols{n},'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',14)
            ylim([0 1.2])
            ylabel('Velocity m/s')
            set(gca,'FontSize',18,'fontWeight','bold')
            
            grid on
            
            subplot(2,1,2)
            plot(t,wrapTo180(theta*180/pi), 'LineWidth',2.0)
            hold on
            plot(t,wrapTo180(attack*180/pi), 'LineWidth',2.0)
            plot(t,wrapTo180(phi*180/pi), 'LineWidth',2.0)
            plot(t([1 end]),[1 1]*90,'k--', 'LineWidth',1.0)
            plot(t([1 end]),[1 1]*-90,'k--', 'LineWidth',1.0)
            plot(max(t)/2,100,symbols{n},'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',28)
            colormap(flipud(parula))
            ylabel('Angle degrees')
            xlabel('time s')
            hold off
            legend({'Pitch','Attack','Glide'},'Location','Best','NumColumns',3)
            %title( [ symbols{n}],'MarkerFaceColor','r','MarkerEdgeColor','black','MarkerSize',14)
            ylim([-100 100])
            set(gca,'YTick',[-90 -45 0 45 90])
            xlim([0 600])
            set(gca,'FontSize',18,'fontWeight','bold')
            grid on
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
        end
        
end

%%

switch internal_or_external
    
    case 1
        
        
    case 2
        switch d_or_l_sweep
            
            case 1
                
                for v=1:length(l)
                    for n=1:length(d)
                        u=2;% identify static value of m 2 = 0.3kg
                        
                        % Load file
                        filename = [txt '_L' num2str(L) '_D' num2str(D) '_A' num2str(A) '_l' num2str(l(v)) '_d' num2str(d(n)) '_m' num2str(m(u)) '.mat'];
                        load(['Results' filesep filename])
                        
                        x = z(:,1); % x-position - horizontal distance travelled
                        vx = z(:,2); % x-velocity - horizontal velocity
                        y = z(:,3); % y-position - depth
                        vy = z(:,4); % y-velocity - vertical velocity
                        theta = z(:,5); % pitch angle
                        vtheta = z(:,6); % pitch angular velocity
                        
                        % derived outputs
                        phi = atan2(vy, vx);
                        attack = wrapToPi(theta - phi);
                        
                        % find the descent path final value
                        I = find( abs(t-300)==min(abs(t-300)));
                        I = I(1)-1;
                        tsample(v,n) = t(I);
                        c_down(v,n) = squeeze(CoE_down_static(:,:,:,v,n,u));
                        v_down(v,n) = sqrt(vx(I)^2+vy(I)^2);
                        phi_down(v,n) = atan2(vy(I), vx(I)).*180/pi;
                        theta_down(v,n) = theta(I).*180/pi;
                        attack_down(v,n) = mean(attack(I-10:I)).*180/pi;
                        weight_force_down(v,n) = squeeze(weight(:,:,:,v,n,u));
                        buoyancy_down_force(v,n) = squeeze(down_buoyancy(:,:,:,v,n,u));
                        pitch_wobble_down(v,n) = squeeze(down_pitch_wobble(:,:,:,v,n,u)).*180/pi;
                        LM_down(v,n) = squeeze(l_m_down(:,:,:,v,n,u));
                        LB_down(v,n) = squeeze(l_b_down(:,:,:,v,n,u));
                        shadow_area_down = abs( ( pi * ( D/2 )^2 + ( 0.1 * A ) ) * cos(phi(I)) + ( ( L * D ) + (A) ) * sin(phi(I)) );
                        [Cd, Cl] = liftdrag(mean(attack(I-10:I)), v_down(v,n));
                        lift_force_down(v,n) = cosd(phi_down(v,n)) * 0.5 * rho * shadow_area_down * v_down(v,n)^2 * Cl;
                        drag_force_down(v,n) = sind(phi_down(v,n)) *0.5 * rho * shadow_area_down * v_down(v,n)^2 * Cd;
                        
                        % up path final values
                        I = length(t);
                        c_up(v,n) = squeeze(CoE_up_static(:,:,:,v,n,u));
                        v_up(v,n) = sqrt(vx(I)^2+vy(I)^2);
                        phi_up(v,n) = atan2(vy(I), vx(I)).*180/pi;
                        theta_up(v,n) = theta(I).*180/pi;
                        attack_up(v,n) = attack(I).*180/pi;
                        weight_force_up(v,n) = squeeze(weight(:,:,:,v,n,u));
                        buoyancy_up_force(v,n) = squeeze(up_buoyancy(:,:,:,v,n,u));
                        pitch_wobble_up(v,n) = squeeze(up_pitch_wobble(:,:,:,v,n,u)).*180/pi;
                        LM_up(v,n) = squeeze(l_m_up(:,:,:,v,n,u));
                        shadow_area_up = abs( ( pi * ( D/2 )^2 + ( 0.1 * A ) ) * cos(phi(I)) + ( ( L * D ) + (A) ) * sin(phi(I)) );
                        [Cd, Cl] = liftdrag(mean(attack(I-10:I)), v_up(v,n));
                        lift_force_up(v,n) = cosd(phi_down(v,n)) * 0.5 * rho * shadow_area_up * v_up(v,n)^2 * Cl;
                        drag_force_up(v,n) = sind(phi_down(v,n)) * 0.5 * rho * shadow_area_up * v_up(v,n)^2 * Cd;
                        
                    end
                end
)
                %
                figure(3424)
                semilogy(c_down(51,:),pitch_wobble_down(51,:),'-.', 'LineWidth',1.5)%d
                % semilogy(median(c_down),median(pitch_wobble_down),'r-', 'LineWidth',5.0)%d
                hold on
                % semilogy((c_down),(pitch_wobble_down), 'LineWidth',1.0)%d
                semilogy(c_down(39,:),pitch_wobble_down(51,:),'-.', 'LineWidth',1.5)%d
                % semilogy(median(c_down),median(pitch_wobble_down),'r-', 'LineWidth',5.0)%d
                semilogy(c_down(26,:),pitch_wobble_down(26,:),'-.', 'LineWidth',1.5)%d
                semilogy(c_down(13,:),pitch_wobble_down(51,:),'-.', 'LineWidth',1.5)%d
                semilogy(c_down(1,:),pitch_wobble_down(1,:),'-.', 'LineWidth',1.5)%d
                xlim([-0.1 0.1])
                ylim([0 100])
                set(gca,'FontSize',18,'fontWeight','bold')
                %                 set(gca,'XTicklabel',[])
                %                 set(gca,'YTick',[0 10 20 30 40])
                grid on
                hold off
                xlabel('C_E, m') %CoE')
                ylabel('Pitch Wobble, degrees') %CoE')
                legend('d=0.05m','d=0.04m','d=0.03m','d=0.02m','d=0.01m')
                
                
                
                % Create textbox
                annotation('textbox',...
                    [0.625479166666668 0.527291452111226 0.220354166666667 0.0895983522142121],...
                    'Color',[1 0 1],...
                    'String','Gliding flight regime',...
                    'FontWeight','bold',...
                    'FontSize',18,...
                    'FitBoxToText','off',...
                    'EdgeColor',[1 1 1]);
                
                % Create doublearrow
                annotation('doublearrow',[0.523958333333333 0.898958333333333],...
                    [0.553038105046344 0.54994850669413],'Color',[1 0 1],'LineWidth',6);
                
                % Create line
                annotation('line',[0.517708333333333 0.518229166666667],...
                    [0.128763130792997 0.91967044284243],'LineWidth',3,'LineStyle','-.');
                
                % Create textbox
                annotation('textbox',...
                    [0.212979166666667 0.821833161688981 0.230208333333333 0.0617919670442842],...
                    'String',{'Bounding flight regime'},...
                    'FontWeight','bold',...
                    'FontSize',18,...
                    'EdgeColor',[1 1 1]);
                
                % Create doublearrow
                annotation('doublearrow',[0.133333333333333 0.514583333333333],...
                    [0.825982492276004 0.82183316168898],'LineWidth',5);
                
                
                %
                figure(156)
                
                subplot(3, 1, 1)
                yyaxis left
                plot(c_down(51,:),phi_down(51,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(51,:),theta_down(51,:),'k-.', 'LineWidth',1.0)%d
                patch([-0.019 -0.5 -0.5 -0.019],[45 45 -130 -130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 -0.019 -0.019 0],[45 45 -130 -130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                                plot(c_down(51,:),pitch_wobble_down(51,:),'--m', 'LineWidth',1.5)%d    
                xlim([-0.1 1.5])
                ylim([-130 45])
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                hold off
                
                yyaxis right
                patch([0.095 2.8 2.8 0.095],[-0.6 -0.6 0.1 0.1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                
                

                
                plot(c_down(51,:), buoyancy_down_force(51,:)- weight_force_down(51,:)+lift_force_down(51,:).*cosd(phi_down(51,:)),':','LineWidth', 1.5)
                
                
                ylim([-0.6 0.1])
                %                 legend('glide angle \phi','C_E = 0, flutter transition unstable flight zone','residue forces down N', 'weight dominates diving transition unstable flight zone', 'location', 'best', 'NumColumns',1)
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                
                axes('pos',[.2 .5 .4 .3])
                imshow('buoyancy_engine_down.png')
                
             
                hold off
                

                
                
                subplot(3, 1, 2)
                yyaxis left
                plot(c_down(26,:),phi_down(26,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(26,:),theta_down(26,:),'k-.', 'LineWidth',1.0)%d
                %                 plot(c_down(26,:),pitch_wobble_down(26,:)*100','g-.', 'LineWidth',1.0)%d
                patch([-0.0207 -0.5 -0.5 -0.0207],[45 45 -130 -130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 -0.0207 -0.0207 0],[45 45 -130 -130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-0.1 1.5])
                ylim([-130 45])
                ylabel('Descent flight angle, degrees')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                hold off
                
                yyaxis right
                %                 plot(c_down(26,:), buoyancy_down_force(26,:)- weight_force_down(26,:)+lift_force_down(26,:)+drag_force_down(26,:),':','LineWidth', 1.5)
                patch([0.207 2.8 2.8 0.207],[-0.6 -0.6 0.1 0.1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                
                hold on
                plot(c_down(26,:), buoyancy_down_force(26,:)- weight_force_down(26,:)+lift_force_down(26,:).*cosd(phi_down(26,:)),':','LineWidth', 1.5)
                
                ylim([-0.6 0.1])
                %                 legend('glide angle \phi','C_E = 0, flutter transition unstable flight zone','residue forces down N', 'weight dominates diving transition unstable flight zone', 'location', 'best', 'NumColumns',1)
                set(gca,'FontSize',18,'fontWeight','bold')
                ylabel('Residual forces, N')
                set(gca,'XTicklabel',[])
                hold off
                
                
                
                
                
                subplot(3, 1, 3)
                yyaxis left
                plot(c_down(1,:),phi_down(1,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(1,:),theta_down(1,:),'k-.', 'LineWidth',1.0)%d
                %                                 plot(c_down(1,:),pitch_wobble_down(1,:)','g-.', 'LineWidth',1.0)%d
                patch([-0.00308 -0.5 -0.5 -0.00308],[45 45 -130 -130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 -0.00308 -0.00308 0],[45 45 -130 -130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                %                 legend(['down m = ' num2str(m(1)) 'kg'],'unstable flight zones', 'location', 'best', 'NumColumns' ,1)
                xlim([-0.1 1.5])
                ylim([-130 45])
                xlabel('C_E, m') %CoE')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                hold off
                
                yyaxis right
                %                 plot(c_down(1,:), buoyancy_down_force(1,:)- weight_force_down(1,:)+lift_force_down(1,:)+drag_force_down(1,:),':','LineWidth', 1.5)
                
                patch([2.6 1.429 1.429 2.6],[-0.6 -0.6 0.1 0.1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                plot(c_down(1,:), buoyancy_down_force(1,:)- weight_force_down(1,:)+lift_force_down(1,:).*cosd(phi_down(1,:)),':','LineWidth', 1.5)
                
                hold off
                
                ylim([-0.6 0.1])
                set(gca,'FontSize',18,'fontWeight','bold')
                
                
                
                %
                
                
                figure(157)
                
                subplot(3, 1, 1)
                yyaxis left
                plot(c_up(51,:),phi_up(51,:),'LineWidth',1.0)%full scuzzy range
                hold on
                plot(c_up(51,:),theta_up(51,:),'k-.', 'LineWidth',1.0)%d
                %                 plot(c_up(51,:),pitch_wobble_up(51,:),'g-.', 'LineWidth',1.0)%d
                patch([0.04343 1.5 1.5 0.04343],[-130 -130 130 130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 0.04343 0.04343 0],[-130 -130 130 130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-1.5 0.1])
                ylim([-100 130])
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                yyaxis right
                
                patch([-2.5 -0.02002 -0.02002 -2.5],[-0.1 -0.1 1 1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                
                hold on
                plot(c_up(51,:), buoyancy_up_force(51,:)- weight_force_up(51,:)+lift_force_up(51,:),':','LineWidth', 1.5)
                ylim([-0.1 1])
                %                 legend('glide angle \phi','C_E = 0, flutter transition unstable flight zone','residue forces down N', 'weight dominates diving transition unstable flight zone', 'location', 'best', 'NumColumns',1)
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                
                
                subplot(3, 1, 2)
                yyaxis left
                hold on
                plot(c_up(26,:),phi_up(26,:),'LineWidth',1.0)%d
                plot(c_up(26,:),theta_up(26,:),'k-.', 'LineWidth',1.0)%d
                %                                 plot(c_up(51,:),pitch_wobble_up(26,:)*100,'g-.', 'LineWidth',1.0)%d
                patch([0.056 0.5 0.5 0.056],[-130 -130 130 130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 0.056 0.056 0],[-130 -130 130 130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                %                 legend(['up m = ' num2str(m(2)) 'kg'],'unstable flight zones', 'location', 'best', 'NumColumns' ,1)
  
                xlim([-1.5 0.1])
                ylim([-100 130])
                ylabel('Ascent flight angle, degrees')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                
                yyaxis right
                patch([-2.5 -0.157 -0.157 -2.5],[-0.1 -0.1 1 1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                plot(c_up(26,:), buoyancy_up_force(26,:)- weight_force_up(26,:)+lift_force_up(26,:),':','LineWidth', 1.5)
                
                
                ylim([-0.1 1])
                legend('glide angle \phi','pitch angle \theta','Futtering flight regime','Bounding flight regime', 'Diving flight regime','Residue forces N', 'location', 'best', 'NumColumns',2)
                set(gca,'FontSize',18,'fontWeight','bold')
                
                ylabel('Residual forces, N')
                set(gca,'XTicklabel',[])
                
                
                
                
                
                
                subplot(3, 1, 3)
                yyaxis left
                plot(c_up(1,:),phi_up(1,:), 'LineWidth',1.0)%d
                hold on
                plot(c_up(1,:),theta_up(1,:),'k-.', 'LineWidth',1.0)%d
                %                                 plot(c_up(1,:),pitch_wobble_up(1,:),'g-.', 'LineWidth',1.0)%d
                patch([0.02809 0.5 0.5 0.02809],[-130 -130 130 130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 0.02809 0.02809 0],[-130 -130 130 130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-1.5 0.1])
                ylim([-100 130])
                xlabel('C_E, m') %CoE')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 set(gca,'XTick',[-2.2 -2.0 -1.8 -1.6 -1.4 -1.2 -1.0 -0.8 -0.6 -0.4 -0.2 0 0.2])
                grid on
                
                yyaxis right
                patch([-3.5 -1.404 -1.404 -3.5],[-0.1 -0.1 1 1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                
                plot(c_up(1,:), buoyancy_up_force(1,:)- weight_force_up(1,:)+lift_force_up(1,:),':','LineWidth', 1.5)
                
                ylim([-0.1 1])
                set(gca,'FontSize',18,'fontWeight','bold')
                

                
%               
                
                figure(156156)
                yyaxis left
                plot(c_down(26,:),phi_down(26,:), 'LineWidth',2)%d
                hold on
                plot(c_down(26,:),theta_down(26,:),'k-.', 'LineWidth',2)%d
                plot(c_down(16,:),pitch_wobble_down(16,:),':r', 'LineWidth',2)%d    
                %                 plot(c_down(26,:),pitch_wobble_down(26,:)*100','g-.', 'LineWidth',1.0)%d
                patch([-0.0207 -0.5 -0.5 -0.0207],[45 45 -130 -130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 -0.0207 -0.0207 0],[45 45 -130 -130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                
                            
                xlim([-0.1 0.25])
                ylim([-130 45])
                ylabel('Descent flight angle, degrees')
                set(gca,'FontSize',22,'fontWeight','bold')
                set(gca,'YTick',[-90 -45 0 45 90])
                xlabel('C_E, m') %CoE')
                
                grid on
                hold off
                
                yyaxis right
                %                 plot(c_down(26,:), buoyancy_down_force(26,:)- weight_force_down(26,:)+lift_force_down(26,:)+drag_force_down(26,:),':','LineWidth', 1.5)
                patch([0.207 2.8 2.8 0.207],[-0.6 -0.6 0.1 0.1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                
                hold on
                plot(c_down(26,:), buoyancy_down_force(26,:)- weight_force_down(26,:)+lift_force_down(26,:).*cosd(phi_down(26,:)),'--m','LineWidth', 2)
                
                ylim([-0.6 0.1])
                legend('glide angle \phi','pitch angle \theta','Pitch Wobble','Futtering flight regime','Bounding flight regime', 'Diving flight regime', 'Residue forces N','location', 'best', 'NumColumns',1)
                set(gca,'FontSize',22,'fontWeight','bold')
                ylabel('Residual forces, N')

                hold off               
                
                %%
                
                
                
                figure(356)
                
                
                %                yyaxis left
                plot(c_down(51,:),d, 'LineWidth',1.0)%d
                
                hold on
                
                plot(c_down(26,:),d, 'LineWidth',1.0)%d
                
                plot(c_down(1,:),d,'k-.', 'LineWidth',1.0)%d
                hold off
                
                
                legend('l = 0.15m','l = 0.10m','l = 0.05m', 'location', 'best', 'NumColumns',2)
                set(gca,'FontSize',12,'fontWeight','bold')
                
                figure(357)
                
                
                %                yyaxis left
                plot(c_up(51,:),d, 'LineWidth',1.0)%d
                
                hold on
                
                plot(c_up(26,:),d, 'LineWidth',1.0)%d
                
                plot(c_up(1,:),d,'k-.', 'LineWidth',1.0)%d
                hold off
                
                
                legend('l = 0.15m','l = 0.10m','l = 0.05m', 'location', 'best', 'NumColumns',2)
                set(gca,'FontSize',12,'fontWeight','bold')
                
                
                
                
                

                
                clearvars x vx y vy theta vtheta I z phi c_up c_down tsample c_down v_down phi_down theta_down attack_down weight_force_down buoyancy_down_force LM_down LB_down shadow_area_down Cd Cl lift_force_down drag_force_down v_up phi_up theta_up attack_up weight_force_up buoyancy_up_force LM_up shadow_area_up lift_force_up drag_force_up
                
                for uu=1:length(m)
                    for nn=1:length(d)
                        vv=26;% variable length (l) 6 = mid position 0.1m
                        
                        % Load file
                        filename = [txt '_L' num2str(L) '_D' num2str(D) '_A' num2str(A) '_l' num2str(l(vv)) '_d' num2str(d(nn)) '_m' num2str(m(uu)) '.mat'];
                        load(['Results' filesep filename])
                        
                        x = z(:,1); % x-position - horizontal distance travelled
                        vx = z(:,2); % x-velocity - horizontal velocity
                        y = z(:,3); % y-position - depth
                        vy = z(:,4); % y-velocity - vertical velocity
                        theta = z(:,5); % pitch angle
                        vtheta = z(:,6); % pitch angular velocity
                        
                        % derived outputs
                        phi = atan2(vy, vx); % glide angle
                        attack = wrapToPi(theta - phi); % angle of attack
                        
                        % down path final value
                        I = find( abs(t-300)==min(abs(t-300)));
                        I = I(1)-1;
                        tsample(uu,nn) = t(I);
                        c_down(uu,nn) = squeeze(CoE_down_static(:,:,:,vv,nn,uu));
                        v_down(uu,nn) = sqrt(vx(I)^2+vy(I)^2);
                        phi_down(uu,nn) = atan2(vy(I), vx(I)).*180/pi;
                        theta_down(uu,nn) = theta(I).*180/pi;
                        attack_down(uu,nn) = mean(attack(I-10:I)).*180/pi;
                        weight_force_down(uu,nn) = squeeze(weight(:,:,:,vv,nn,uu));
                        buoyancy_down_force(uu,nn) = squeeze(down_buoyancy(:,:,:,vv,nn,uu));
                        LM_down(uu,nn) = squeeze(l_m_down(:,:,:,vv,nn,uu));
                        LB_down(uu,nn) = squeeze(l_b_down(:,:,:,vv,nn,uu));
                        shadow_area_down = abs( ( pi * ( D/2 )^2 + ( 0.1 * A ) ) * cos(phi(I)) + ( ( L * D ) + (A) ) * sin(phi(I)) );
                        [Cd, Cl] = liftdrag(mean(attack(I-10:I)), v_down(uu,nn));
                        lift_force_down(uu,nn) = cosd(phi_down(uu,nn)) * 0.5 * rho * shadow_area_down * v_down(uu,nn)^2 * Cl;
                        drag_force_down(uu,nn) = sind(phi_down(uu,nn)) *0.5 * rho * shadow_area_down * v_down(uu,nn)^2 * Cd;
                        
                        % up path final values
                        I = length(t);
                        c_up(uu,nn) = squeeze(CoE_up_static(:,:,:,vv,nn,uu));
                        v_up(uu,nn) = sqrt(vx(I)^2+vy(I)^2);
                        phi_up(uu,nn) = atan2(vy(I), vx(I)).*180/pi;
                        theta_up(uu,nn) = theta(I).*180/pi;
                        attack_up(uu,nn) = attack(I).*180/pi;
                        weight_force_up(uu,nn) = squeeze(weight(:,:,:,vv,nn,uu));
                        buoyancy_up_force(uu,nn) = squeeze(up_buoyancy(:,:,:,vv,nn,uu));
                        LM_up(uu,nn) = squeeze(l_m_up(:,:,:,vv,nn,uu));
                        shadow_area_up = abs( ( pi * ( D/2 )^2 + ( 0.1 * A ) ) * cos(phi(I)) + ( ( L * D ) + (A) ) * sin(phi(I)) );
                        [Cd, Cl] = liftdrag(mean(attack(I-10:I)), v_up(uu,nn));
                        lift_force_up(uu,nn) = cosd(phi_down(uu,nn)) * 0.5 * rho * shadow_area_up * v_up(uu,nn)^2 * Cl;
                        drag_force_up(uu,nn) = sind(phi_down(uu,nn)) * 0.5 * rho * shadow_area_up * v_up(uu,nn)^2 * Cd;
                    end
                end
                
                
                
                
                
                
                
                
                
                %%
                
                figure(56)
                
                subplot(3, 1, 1)
           
                yyaxis left
                plot(c_down(3,:),phi_down(3,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(3,:),theta_down(3,:),'k-.', 'LineWidth',1.0)%d
                patch([-0.02203 -0.5 -0.5 -0.02203],[45 45 -130 -130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 -0.02203 -0.02203 0],[45 45 -130 -130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-0.1 0.6])
                ylim([-130 45])
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                hold off
                
                yyaxis right
                patch([0.1315 2.8 2.8 0.1315],[-0.6 -0.6 0.1 0.1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                plot(c_down(3,:), buoyancy_down_force(3,:)- weight_force_down(3,:)+lift_force_down(3,:),':','LineWidth', 1.5)
                ylim([-0.6 0.1])
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                axes('pos',[.1 .6 .5 .3])
                hold off
                
                subplot(3, 1, 2)
                yyaxis left
                plot(c_down(2,:),phi_down(2,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(2,:),theta_down(2,:),'k-.', 'LineWidth',1.0)%d
                patch([-0.0207 -0.5 -0.5 -0.0207],[45 45 -130 -130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 -0.0207 -0.0207 0],[45 45 -130 -130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-0.1 0.6])
                ylim([-130 45])
                ylabel('Descent flight angle, degrees')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                hold off
                
                yyaxis right
                patch([0.207 2.8 2.8 0.207],[-0.6 -0.6 0.1 0.1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                plot(c_down(2,:), buoyancy_down_force(2,:)- weight_force_down(2,:)+lift_force_down(2,:),':','LineWidth', 1.5)
                ylim([-0.6 0.1])
                set(gca,'FontSize',18,'fontWeight','bold')
                ylabel('Residual forces, N')
                set(gca,'XTicklabel',[])
                hold off
                
                
                subplot(3, 1, 3)
                yyaxis left
                plot(c_down(1,:),phi_down(1,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(1,:),theta_down(1,:),'k-.', 'LineWidth',1.0)%d
                patch([-0.01368 -0.5 -0.5 -0.01368],[45 45 -130 -130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 -0.01368 -0.01368 0],[45 45 -130 -130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-0.1 0.6])
                ylim([-130 45])
                xlabel('C_E, m') %CoE')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                hold off
                
                yyaxis right
                patch([2.6 0.4787 0.4787 2.6],[-0.6 -0.6 0.1 0.1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                plot(c_down(1,:), buoyancy_down_force(1,:)- weight_force_down(1,:)+lift_force_down(1,:),':','LineWidth', 1.5)
                hold off
                ylim([-0.6 0.1])
                legend('glide angle \phi','pitch angle \theta','Futtering flight regime','Bounding flight regime', 'Diving flight regime','Residue forces N', 'location', 'best', 'NumColumns',2)
                set(gca,'FontSize',18,'fontWeight','bold')
                
                
                

                %%
                
                
                figure(57)
                
                subplot(3, 1, 1)
                yyaxis left
                plot(c_up(3,:),phi_up(3,:),'LineWidth',1.0)%full scuzzy range
                hold on
                plot(c_up(3,:),theta_up(3,:),'k-.', 'LineWidth',1.0)%d
                patch([0.04692 1.5 1.5 0.04692],[-130 -130 130 130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 0.04692 0.04692 0],[-130 -130 130 130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-0.5 0.1])
                ylim([-100 130])
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                yyaxis right
                patch([-2.5 -0.08154 -0.08154 -2.5],[-0.1 -0.1 1 1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                plot(c_up(3,:), buoyancy_up_force(3,:)- weight_force_up(3,:)+lift_force_up(3,:),':','LineWidth', 1.5)
                
                
                ylim([-0.1 1])
                %                 legend('glide angle \phi','C_E = 0, flutter transition unstable flight zone','residue forces down N', 'weight dominates diving transition unstable flight zone', 'location', 'best', 'NumColumns',1)
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                
                
                subplot(3, 1, 2)
                yyaxis left
                hold on
                plot(c_up(2,:),phi_up(2,:),'LineWidth',1.0)%d
                plot(c_up(2,:),theta_up(2,:),'k-.', 'LineWidth',1.0)%d
                patch([0.056 0.5 0.5 0.056],[-130 -130 130 130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 0.056 0.056 0],[-130 -130 130 130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                %                 legend(['up m = ' num2str(m(2)) 'kg'],'unstable flight zones', 'location', 'best', 'NumColumns' ,1)
                xlim([-0.5 0.1])
                ylim([-100 130])
                ylabel('Ascent flight angle, degrees')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'XTicklabel',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                grid on
                
                yyaxis right
                patch([-2.5 -0.157 -0.157 -2.5],[-0.1 -0.1 1 1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                
                plot(c_up(2,:), buoyancy_up_force(2,:)- weight_force_up(2,:)+lift_force_up(2,:),':','LineWidth', 1.5)
                
                ylim([-0.1 1])
                legend('glide angle \phi','pitch angle \theta','Futtering flight regime','Bounding flight regime', 'Diving flight regime','Residue forces N', 'location', 'best', 'NumColumns',2)
                set(gca,'FontSize',18,'fontWeight','bold')
                
                ylabel('Residual forces, N')
                set(gca,'XTicklabel',[])
                
                
                
                
                
                
                subplot(3, 1, 3)
                yyaxis left
                plot(c_up(1,:),phi_up(1,:), 'LineWidth',1.0)%d
                hold on
                plot(c_up(1,:),theta_up(1,:),'k-.', 'LineWidth',1.0)%d
                patch([0.06368 0.5 0.5 0.06368],[-130 -130 130 130],'b','FaceAlpha',0.1,'EdgeAlpha',0.0)
                patch([0 0.06368 0.06368 0],[-130 -130 130 130],'g','FaceAlpha',0.1,'EdgeAlpha',0.0)
                xlim([-0.5 0.1])
                ylim([-100 130])
                xlabel('C_E, m') %CoE')
                set(gca,'FontSize',18,'fontWeight','bold')
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 set(gca,'XTick',[-2.2 -2.0 -1.8 -1.6 -1.4 -1.2 -1.0 -0.8 -0.6 -0.4 -0.2 0 0.2])
                grid on
                
                yyaxis right
                
                patch([-3.5 -0.4287 -0.4287 -3.5],[-0.1 -0.1 1 1],'r','FaceAlpha',0.1,'EdgeAlpha',0.0)
                hold on
                plot(c_up(1,:), buoyancy_up_force(1,:)- weight_force_up(1,:)+lift_force_up(1,:),':','LineWidth', 1.5)
                
                ylim([-0.1 1])
                set(gca,'FontSize',18,'fontWeight','bold')
                
                
          
                
                
                
                
                %%
                figure(600)
                imagesc(d,l,c_up')
                colormap(parula)
                c = colorbar;
                c.Label.String = 'CoE, m';
                caxis([-0.2 0.2])
                %%
                
                figure(700)
                subplot(3, 1, 1)
                plot(c_down(3,:),phi_down(3,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(3,:),theta_down(3,:), 'LineWidth',1.0)%d
                plot(c_down(3,:),theta_down(3,:)-phi_down(3,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                xlim([-0.003 0.003])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('C_E, m')
                ylabel('Angle, degree')
                %   legend('Glide Angle descending', 'Pitch Angle descending', 'Angle of attack descending')
                hold off
                subplot(3, 1, 2)
                plot(c_down(2,:),phi_down(2,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(2,:),theta_down(2,:), 'LineWidth',1.0)%d
                plot(c_down(2,:),theta_down(2,:)-phi_down(2,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                xlim([-0.003 0.005])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('C_E, m')
                ylabel('Angle, degree')
                hold off
                
                subplot(3, 1, 3)
                plot(c_down(1,:),phi_down(1,:), 'LineWidth',1.0)%d
                hold on
                plot(c_down(1,:),theta_down(1,:), 'LineWidth',1.0)%d
                plot(c_down(1,:),theta_down(1,:)-phi_down(1,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                xlim([-0.003 0.022])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('C_E, m')
                ylabel('Angle, degree')
                %               legend('Glide Angle descending', 'Pitch Angle descending', 'Angle of attack descending')
                hold off
                legend('Glide Angle descending', 'Pitch Angle descending', 'Angle of attack descending', 'location', 'best', 'NumColumns' ,3)
                
                
                
                
                
                
                
                
                
                figure(701)
                subplot(3, 1, 1)
                
                plot(c_up(3,:),phi_up(3,:), 'LineWidth',1.0)%d
                hold on
                plot(c_up(3,:),theta_up(3,:), 'LineWidth',1.0)%d
                plot(c_up(3,:),theta_up(3,:)-phi_up(3,:), 'LineWidth',1.0)%d
                ylim([-130 130])
                xlim([-0.003 0.003])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent')
                grid on
                xlabel('C_E, m')
                %                 ylabel('Angle, degree')
                %                 legend('Glide angle ascending', 'Pitch angle ascending', 'Angle of attack ascending')
                hold off
                
                subplot(3, 1, 2)
                plot(c_up(2,:),phi_up(2,:), 'LineWidth',1.0)%d
                hold on
                plot(c_up(2,:),theta_up(2,:), 'LineWidth',1.0)%d
                plot(c_up(2,:),theta_up(2,:)-phi_up(2,:), 'LineWidth',1.0)%d
                ylim([-130 130])
                xlim([-0.005 0.003])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent')
                grid on
                xlabel('C_E, m')
                ylabel('Angle, degree')
                hold off
                
                subplot(3, 1, 3)
                plot(c_up(1,:),phi_up(1,:), 'LineWidth',1.0)%d
                hold on
                plot(c_up(1,:),theta_up(1,:), 'LineWidth',1.0)%d
                plot(c_up(1,:),theta_up(1,:)-phi_up(1,:), 'LineWidth',1.0)%d
                ylim([-130 130])
                xlim([-0.022 0.003])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent')
                grid on
                xlabel('C_E, m')
                %                 ylabel('Angle, degree')
                %                 legend('Glide angle ascending', 'Pitch angle ascending', 'Angle of attack ascending')
                hold off
                legend('Glide angle ascending', 'Pitch angle ascending', 'Angle of attack ascending', 'location', 'best', 'NumColumns' ,3)
                
                %%
                
                figure(800)
                subplot(3, 1, 1)
                plot(c_down(3,:),phi_down(3,:), 'LineWidth',1.0)%d
                hold on
                plot(-LM_down(3,:),phi_down(3,:), 'LineWidth',1.0)%d
                plot(-LB_down(3,:),phi_down(3,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                %xlim([-0.003 0.003])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('Centres, m')
                ylabel('Angle, degree')
                %   legend('Glide Angle descending', 'Pitch Angle descending', 'Angle of attack descending')
                hold off
                
                subplot(3, 1, 2)
                plot(-c_down(2,:),phi_down(2,:), 'LineWidth',1.0)%d
                hold on
                plot(LM_down(2,:),phi_down(2,:), 'LineWidth',1.0)%d
                plot(LB_down(2,:),phi_down(2,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                %xlim([-0.003 0.005])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('Centres, m')
                ylabel('Angle, degree')
                hold off
                
                subplot(3, 1, 3)
                plot(-c_down(1,:),phi_down(1,:), 'LineWidth',1.0)%d
                hold on
                plot(LM_down(1,:),phi_down(1,:), 'LineWidth',1.0)%d
                plot(LB_down(1,:),phi_down(1,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                %xlim([-0.003 0.022])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('Centres, m')
                ylabel('Angle, degree')
                %               legend('Glide Angle descending', 'Pitch Angle descending', 'Angle of attack descending')
                hold off
                legend('C_E', 'C_M', 'C_B', 'location', 'best', 'NumColumns' ,3)
                %%
                
                LB_up = LM_up*0;
                figure(801)
                subplot(3, 1, 1)
                
                plot(-c_up(3,:),phi_up(3,:), 'LineWidth',1.0)%d
                hold on
                plot(LM_up(3,:),phi_up(3,:), 'LineWidth',1.0)%d
                plot(LB_up(3,:),phi_up(3,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                %xlim([-0.003 0.003])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('Centres, m')
                ylabel('Angle, degree')
                %   legend('Glide Angle descending', 'Pitch Angle descending', 'Angle of attack descending')
                hold off
                
                subplot(3, 1, 2)
                plot(-c_up(2,:),phi_up(2,:), 'LineWidth',1.0)%d
                hold on
                plot(LM_up(2,:),phi_up(2,:), 'LineWidth',1.0)%d
                plot(LB_up(2,:),phi_up(2,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                %xlim([-0.003 0.005])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('Centres, m')
                ylabel('Angle, degree')
                hold off
                
                subplot(3, 1, 3)
                plot(-c_up(1,:),phi_up(1,:), 'LineWidth',1.0)%d
                hold on
                plot(LM_up(1,:),phi_up(1,:), 'LineWidth',1.0)%d
                plot(LB_up(1,:),phi_up(1,:), 'LineWidth',1.0)%d
                
                ylim([-130 130])
                %xlim([-0.003 0.022])
                set(gca,'FontSize',14,'fontWeight','bold')
                %                                 set(gca,'XTick',[])
                set(gca,'YTick',[-90 -45 0 45 90])
                %                 title('Descent Glide Angle')
                grid on
                xlabel('Centres, m')
                ylabel('Angle, degree')
                %               legend('Glide Angle descending', 'Pitch Angle descending', 'Angle of attack descending')
                hold off
                legend('C_E', 'C_M', 'C_B', 'location', 'best', 'NumColumns' ,3)
                
             
                
                
                
                %%
                
                figure(100)% CE and forces
                %                 plot(c_down(1,:), weight_force_down(1,:))
                
                plot(c_down(1,:), buoyancy_down_force(1,:)- weight_force_down(1,:)+lift_force_down(1,:)+drag_force_down(1,:))
                %                 plot(c_down(1,:), lift_force_down(1,:))
                
                hold on
                
                plot(c_down(2,:), buoyancy_down_force(2,:)- weight_force_down(2,:)+lift_force_down(2,:)+drag_force_down(2,:))
                
                plot(c_down(3,:), buoyancy_down_force(3,:)- weight_force_down(3,:)+lift_force_down(3,:)+drag_force_down(3,:))
                %                 plot(c_down(1,:), drag_force_down(1,:))
                
                figure(101)% CE and forces
                %                 plot(c_up(1,:), weight_force_up(1,:))
                
                plot(c_up(1,:), buoyancy_up_force(1,:)- weight_force_up(1,:)+lift_force_up(1,:)+drag_force_up(1,:))
                %                 plot(c_up(1,:), lift_force_up(1,:))
                
                hold on
                
                plot(c_up(2,:), buoyancy_up_force(2,:)- weight_force_up(2,:)+lift_force_up(2,:)+drag_force_up(2,:))
                
                plot(c_up(3,:), buoyancy_up_force(3,:)- weight_force_up(3,:)+lift_force_up(3,:)+drag_force_up(3,:))
                %                 plot(c_up(1,:), drag_force_up(1,:))
                
                
                
                %%
            case 2
                
                for v=1:length(d)
                    for n=1:length(l)
                        u=2;% static value of m
                        
                        % Load file
                        filename = [txt '_L' num2str(L) '_D' num2str(D) '_A' num2str(A) '_l' num2str(l(n)) '_d' num2str(d(v)) '_m' num2str(m(u)) '.mat'];
                        load(['Results' filesep filename])
                        
                        x = z(:,1); % x-position - horizontal distance travelled
                        vx = z(:,2); % x-velocity - horizontal velocity
                        y = z(:,3); % y-position - depth
                        vy = z(:,4); % y-velocity - vertical velocity
                        theta = z(:,5); % pitch angle
                        vtheta = z(:,6); % pitch angular velocity
                        
                        % derived outputs
                        phi = atan2(vy, vx); % glide angle
                        attack = wrapToPi(theta - phi); % angle of attack
                        
                        % find the descent path final value
                        I = find( abs(t-300)==min(abs(t-300)));
                        I = I(1)-1;
                        tsample(v,n) = t(I);
                        c_down(v,n) = squeeze(CoE_down_static(:,:,:,n,v,u));
                        v_down(v,n) = sqrt(vx(I)^2+vy(I)^2);
                        phi_down(v,n) = atan2(vy(I), vx(I)).*180/pi;
                        theta_down(v,n) = theta(I).*180/pi;
                        attack_down(v,n) = mean(attack(I-10:I)).*180/pi;
                        weight_force_down(v,n) = squeeze(weight(:,:,:,n,v,u));
                        buoyancy_down_force(v,n) = squeeze(down_buoyancy(:,:,:,n,v,u));
                        LM_down(v,n) = squeeze(l_m_down(:,:,:,n,v,u));
                        LB_down(v,n) = squeeze(l_b_down(:,:,:,n,v,u));
                        shadow_area_down = abs( ( pi * ( D/2 )^2 + ( 0.1 * A ) ) * cos(phi(I)) + ( ( L * D ) + (A) ) * sin(phi(I)) );
                        [Cd, Cl] = liftdrag(mean(attack(I-10:I)), A, v_down(v,n));
                        lift_force_down(v,n) = cosd(phi_down(v,n)) * 0.5 * rho * shadow_area_down * v_down(v,n)^2 * Cl;
                        drag_force_down(v,n) = sind(phi_down(v,n)) *0.5 * rho * shadow_area_down * v_down(v,n)^2 * Cd;
                        
                        % up path final values
                        I = length(t);
                        c_up(v,n) = squeeze(CoE_up_static(:,:,:,n,v,u));
                        v_up(v,n) = sqrt(vx(I)^2+vy(I)^2);
                        phi_up(v,n) = atan2(vy(I), vx(I)).*180/pi;
                        theta_up(v,n) = theta(I).*180/pi;
                        attack_up(v,n) = attack(I).*180/pi;
                        weight_force_up(v,n) = squeeze(weight(:,:,:,n,v,u));
                        buoyancy_up_force(v,n) = squeeze(up_buoyancy(:,:,:,n,v,u));
                        LM_up(v,n) = squeeze(l_m_up(:,:,:,n,v,u));
                        shadow_area_up = abs( ( pi * ( D/2 )^2 + ( 0.1 * A ) ) * cos(phi(I)) + ( ( L * D ) + (A) ) * sin(phi(I)) );
                        [Cd, Cl] = liftdrag(mean(attack(I-10:I)), A, v_up(v,n));
                        lift_force_up(v,n) = cosd(phi_down(v,n)) * 0.5 * rho * shadow_area_up * v_up(v,n)^2 * Cl;
                        drag_force_up(v,n) = sind(phi_down(v,n)) * 0.5 * rho * shadow_area_up * v_up(v,n)^2 * Cd;
                    end
                end
                
                %%
                figure(60)
                plot(c_down,l, 'LineWidth',2.0)%d
                grid on
                legend(['d = ' num2str(d(1)) 'm'], ['d = ' num2str(d(2)) 'm'], ['d = ' num2str(d(3)) 'm'], ['d = ' num2str(d(4)) 'm'], ['d = ' num2str(d(5)) 'm'], 'location', 'best')
                
                figure(61)
                subplot(2, 1, 1)
                plot(c_down(1,:),v_down(1,:),'g', 'LineWidth',2.0)%d
                hold on
                plot(c_down(2,:),v_down(2,:),'m', 'LineWidth',2.0)%d
                plot(c_down(3,:),v_down(3,:),'k', 'LineWidth',2.0)%d
                plot(c_down(4,:),v_down(4,:),'r', 'LineWidth',2.0)%d
                plot(c_down(5,:),v_down(5,:),'b', 'LineWidth',2.0)%d
                legend(['d = ' num2str(d(1)) 'm'], ['d = ' num2str(d(2)) 'm'], ['d = ' num2str(d(3)) 'm'], ['d = ' num2str(d(4)) 'm'], ['d = ' num2str(d(5)) 'm'], 'location', 'best')
                grid on
                xlabel('CoE, m')
                ylabel('Velocity, m/s')
                xlim([-0.1 1])
                ylim([0 1.4])
                set(gca,'FontSize',18,'fontWeight','bold')
                title('Descent velocity vs CoE. m = 0.2 kg. Fixed L, D and A')
                
                subplot(2, 1, 2)
                plot(c_down(1,:),weight_force_down(1,:) + drag_force_down(1,:),'g', 'LineWidth',2.0)%d
                hold on
                plot(c_down(1,:),buoyancy_down_force(1,:) + lift_force_down(1,:),':g', 'LineWidth',2.0)%d
                plot(c_down(2,:),weight_force_down(2,:) + drag_force_down(2,:),'m', 'LineWidth',2.0)%d
                plot(c_down(2,:),buoyancy_down_force(2,:) + lift_force_down(2,:),':m', 'LineWidth',2.0)%d
                plot(c_down(3,:),weight_force_down(3,:) + drag_force_down(3,:),'k', 'LineWidth',2.0)%d
                plot(c_down(3,:),buoyancy_down_force(3,:) + lift_force_down(3,:),':k', 'LineWidth',2.0)%d
                plot(c_down(4,:),weight_force_down(4,:) + drag_force_down(4,:),'r', 'LineWidth',2.0)%d
                plot(c_down(4,:),buoyancy_down_force(4,:) + lift_force_down(4,:),':r', 'LineWidth',2.0)%d
                plot(c_down(5,:),weight_force_down(5,:) + drag_force_down(5,:),'b', 'LineWidth',2.0)%d
                plot(c_down(5,:),buoyancy_down_force(5,:) + lift_force_down(5,:),':b', 'LineWidth',2.0)%d
                grid on
                xlabel('CoE, m') %CoE')
                ylabel('Force, N')
                legend(['Weight Force. d = ' num2str(d(1)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(1)) 'm'],['Weight Force. d = ' num2str(l(2)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(2)) 'm'],['Weight Force. d = ' num2str(d(3)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(3)) 'm'],['Weight Force. d = ' num2str(d(4)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(4)) 'm'],['Weight Force. d = ' num2str(d(5)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(5)) 'm'], 'location', 'best', 'NumColumns', 2)
                xlim([-0.1 1])
                set(gca,'FontSize',18,'fontWeight','bold')
                title('Descent forces vs CoE. m = 0.2 kg. Fixed L, D and A')
                
                figure(62)
                subplot(2, 1, 1)
                plot(c_up(1,:),v_up(1,:),'g', 'LineWidth',2.0)%d
                hold on
                plot(c_up(2,:),v_up(2,:),'m', 'LineWidth',2.0)%d
                plot(c_up(3,:),v_up(3,:),'k', 'LineWidth',2.0)%d
                plot(c_up(4,:),v_up(4,:),'r', 'LineWidth',2.0)%d
                plot(c_up(5,:),v_up(5,:),'b', 'LineWidth',2.0)%d
                legend(['d = ' num2str(d(1)) 'm'], ['d = ' num2str(d(2)) 'm'], ['d = ' num2str(d(3)) 'm'], ['d = ' num2str(d(4)) 'm'], ['d = ' num2str(d(5)) 'm'], 'location', 'best')
                grid on
                xlabel('CoE, m')
                ylabel('Velocity, m/s')
                xlim([-1 0.1])
                ylim([0 1.4])
                set(gca,'FontSize',18,'fontWeight','bold')
                title('Ascent velocity vs CoE. m = 0.2 kg. Fixed L, D and A')
                
                subplot(2, 1, 2)
                plot(c_up(1,:),weight_force_up(1,:) + drag_force_up(1,:),'g', 'LineWidth',2.0)%d
                hold on
                plot(c_up(1,:),buoyancy_up_force(1,:) + lift_force_up(1,:),':g', 'LineWidth',2.0)%d
                plot(c_up(2,:),weight_force_up(2,:) + drag_force_up(2,:),'m', 'LineWidth',2.0)%d
                plot(c_up(2,:),buoyancy_up_force(2,:) + lift_force_up(2,:),':m', 'LineWidth',2.0)%d
                plot(c_up(3,:),weight_force_up(3,:) + drag_force_up(3,:),'k', 'LineWidth',2.0)%d
                plot(c_up(3,:),buoyancy_up_force(3,:) + lift_force_up(3,:),':k', 'LineWidth',2.0)%d
                plot(c_up(4,:),weight_force_up(4,:) + drag_force_up(4,:),'r', 'LineWidth',2.0)%d
                plot(c_up(4,:),buoyancy_up_force(4,:) + lift_force_up(4,:),':r', 'LineWidth',2.0)%d
                plot(c_up(5,:),weight_force_up(5,:) + drag_force_up(5,:),'b', 'LineWidth',2.0)%d
                plot(c_up(5,:),buoyancy_up_force(5,:) + lift_force_up(5,:),':b', 'LineWidth',2.0)%d
                grid on
                xlabel('CoE, m') %CoE')
                ylabel('Force, N')
                legend(['Weight Force. d = ' num2str(d(1)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(1)) 'm'],['Weight Force. d = ' num2str(d(2)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(2)) 'm'],['Weight Force. d = ' num2str(d(3)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(3)) 'm'],['Weight Force. d = ' num2str(d(4)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(4)) 'm'],['Weight Force. d = ' num2str(d(5)) 'm'],['Buoyancy and Lift forces. d = ' num2str(d(5)) 'm'], 'location', 'best', 'NumColumns', 2)
                xlim([-1 0.1])
                set(gca,'FontSize',18,'fontWeight','bold')
                title('Ascent forces vs CoE. m = 0.2 kg. Fixed L, D and A')
                
                figure(63)
                subplot(2, 1, 1)
                plot(c_down(1,:),LM_down(1,:),'g', 'LineWidth',2.0)%d
                hold on
                plot(c_down(1,:),LB_down(1,:),':g', 'LineWidth',2.0)%d
                plot(c_down(2,:),LM_down(2,:),'m', 'LineWidth',2.0)%d
                plot(c_down(2,:),LB_down(2,:),':m', 'LineWidth',2.0)%d
                plot(c_down(3,:),LM_down(3,:),'k', 'LineWidth',2.0)%d
                plot(c_down(3,:),LB_down(3,:),':k', 'LineWidth',2.0)%d
                plot(c_down(4,:),LM_down(4,:),'r', 'LineWidth',2.0)%d
                plot(c_down(4,:),LB_down(4,:),':r', 'LineWidth',2.0)%d
                plot(c_down(5,:),LM_down(5,:),'b', 'LineWidth',2.0)%d
                plot(c_down(5,:),LB_down(5,:),':b', 'LineWidth',2.0)%d
                grid on
                xlabel('CoE, m')
                ylabel('Distance, m')
                legend(['l_m. d = ' num2str(d(1)) 'm'],['l_b. d = ' num2str(d(1)) 'm'],['l_m. d = ' num2str(d(2)) 'm'],['l_b. d = ' num2str(d(2)) 'm'],['l_m. d = ' num2str(d(3)) 'm'],['l_b. d = ' num2str(d(3)) 'm'],['l_m. d = ' num2str(d(4)) 'm'],['l_b. d = ' num2str(d(4)) 'm'],['l_m. d = ' num2str(d(5)) 'm'],['l_b. d = ' num2str(d(5)) 'm'], 'location', 'best', 'NumColumns', 2)
                xlim([-0.1 1])
                set(gca,'FontSize',18,'fontWeight','bold')
                title('Descent LM LB vs CoE. m = 0.2 kg. Fixed L, D and A')
                
                
                subplot(2, 1, 2)
                plot(c_up(1,:),LM_up(1,:),'g', 'LineWidth',2.0)%d
                hold on
                plot(c_up(2,:),LM_up(2,:),'m', 'LineWidth',2.0)%d
                plot(c_up(3,:),LM_up(3,:),'k', 'LineWidth',2.0)%d
                plot(c_up(4,:),LM_up(4,:),'r', 'LineWidth',2.0)%d
                plot(c_up(5,:),LM_up(5,:),'b', 'LineWidth',2.0)%d
                legend(['d = ' num2str(d(1)) 'm'], ['d = ' num2str(d(2)) 'm'], ['d = ' num2str(d(3)) 'm'], ['d = ' num2str(d(4)) 'm'], ['d = ' num2str(d(5)) 'm'], 'location', 'best')
                grid on
                xlabel('CoE, m')
                ylabel('Distance, m')
                xlim([-1 0.1])
                set(gca,'FontSize',18,'fontWeight','bold')
                title('Ascent LM vs CoE. m = 0.2 kg. Fixed L, D and A')
                
                figure(64)
                plot(c_down(1,:),phi_down(1,:),'r', 'LineWidth',2.0)%d
                hold on
                plot(c_up(1,:),phi_up(1,:),':r', 'LineWidth',2.0)%d
                plot(c_down(2,:),phi_down(2,:),'m', 'LineWidth',2.0)%d
                plot(c_up(2,:),phi_up(2,:),':m', 'LineWidth',2.0)%d
                plot(c_down(3,:),phi_down(3,:),'k', 'LineWidth',2.0)%d
                plot(c_up(3,:),phi_up(3,:),':k', 'LineWidth',2.0)%d
                plot(c_down(4,:),phi_down(4,:),'r', 'LineWidth',2.0)%d
                plot(c_up(4,:),phi_up(4,:),':r', 'LineWidth',2.0)%d
                plot(c_down(5,:),phi_down(5,:),'b', 'LineWidth',2.0)%d
                plot(c_up(5,:),phi_up(5,:),':b', 'LineWidth',2.0)%d
                legend(['phi down. d = ' num2str(d(1)) 'm'],['phi up. d = ' num2str(d(1)) 'm'],['phi down. d = ' num2str(d(2)) 'm'],['phi up. d = ' num2str(d(2)) 'm'],['phi down. d = ' num2str(d(3)) 'm'],['phi up. d = ' num2str(d(3)) 'm'],['phi down. d = ' num2str(d(4)) 'm'],['phi up. d = ' num2str(d(4)) 'm'],['phi down. d = ' num2str(d(5)) 'm'],['phi up. d = ' num2str(d(5)) 'm'], 'location', 'best', 'NumColumns', 2)
                xlim([-1 1])
        end
        
end