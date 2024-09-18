clear

%% Define sweep scope (design space too big to sweep all parameters so split internal and external)

combo = 3;
% 1 - external parameters sweep (L, D and A) with internal parameters fixed
% 2 - internal sweep (l, d and m) with external parameters fixed
% 3 - external sample (vary L, D and A at a unique design point)
% 4 - internal sample (vary l, d and m at a unique design point)

switch combo
    
    case 1
        % External parameter sweep
        L = [0.6:0.01:1.2]; % Hull length, m
        D = [0.03:0.001:0.1];% Hull diameter, m
        A = [0.04 0.08 0.12 0.16]; % Wing area, m^2
        
    case 2
        % Internal parameter sweep
        l = [0.05:0.002:0.15]; % Engine length, m
        d = [0.01:0.0002:0.05]; % Engine diameter, m
        m = [0.2 0.3 0.4]; % Moving ballast mass, kg
        
    case 3
        % External sample
        L = 1.0;
        D = 6.15e-2;
        A = 0.07;
        
    case 4
        % Internal sample
        l = 0.1;
        d = 0.03;
        m = 0.3;
        
end

switch combo
    case {1,3}
        % External parameter sweep - fixed values for combo 1 and 3
        l = 0.1;
        d = 0.027;
        m = 0.30;
        
    case {2,4}
        % Internal parameter sweep - fixed values for combo 2 and 4
        L = 0.8;%0.9;
        D = 0.05;
        A = 0.08;
        
end

switch combo
    case {1,2,5}
        plotresults = 0;
        saveresults = 1;
    case {3,4}
        plotresults = 1;
        saveresults = 0;
end

s = [0 1]; % max and min of buoyancy engine travel limits

% Other glider parameters
E_0 = 0.2; % location of moving ballast mass (m) positioned on linear actuator/moving assembly
% This is eliminated as m moves relative to M - se paper for further details

g = 9.81;
rho = 1000;

t_act = 25; % time that actuator takes to operate from min to max or vice verca (real world = 25s)

tmax = 400; % total deployment time

trigger_1 = tmax/4; % time when linear actuator operated at depth to return glider to surface


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
                        
                        % These are derived but fixed
                        
                        % Total mass of glider is calculated when buoyancy engine is at half stroke
                        % Buoyancy balances Weight to achieve neutral
                        % buoyancy and (by balancing M against m) zero pitch
                        Vmax = pi*Li*(Di/2)^2; % Total volume of glider body
                        V_engine_neutral = pi*(li/2)*(di/2).^2; % volume of buoyany engine in the neutral (half full/empty) state
                        
                        % Total mass - this is the amount of mass of the
                        % glider at the neutral condition
                        M_total = (Vmax-V_engine_neutral) * rho;
                        
                        % Mass of glider minus moving (fixed value) mass
                        M = M_total - mi;
                        
                        % Weight of glider minus moving mass
                        W = M * g;
                        
                        % Weight of moving mass
                        w = mi * g;
                        
                        % Geometric centre (Fixed)
                        x0 = [0; 0];
                        
                        % Centre of drag (Fixed) - see paper for justification
                        xD = [-0.025 * Li; 0];%[0;0];%
                        
                        % Centre of lift (Fixed) - achieved by use of symmetrical NACA0012
                        % and positioning wing leading edge at x0 with max chord aft of this at quater L - see paper
                        xL = [-0.025 *Li; 0];%[0;0];%
                        
                        
                        %NACA0012 profile - quater chord point is centre
                        %pressure and aerodynamic centre - Anderson,
                        %fundemetals of aerodynamics, 6th ed page 355
                        %% These are derived but variable, depending on stroke
                        
                        % moving mass
                        xm_neutral = [(E_0  + 0.5 * li - Li/2) ; 0]; %fixed (buoyacy engine half full/empty)
                        
                        xm_down = [(E_0  + li - Li/2) ; 0]; % fully descending condition (buoyacy engine full)
                        xm_up = [(E_0 - Li/2) ; 0]; % fully ascending condition (buoyacy engine empty)
                        
                        % Buoyancy
                        Bmax = pi * (Di/2).^2 * Li * rho * g; %fixed (maximum buoyancy - hull with buoyancy engine empty)
                        B_neutral = Bmax - pi* (di/2).^2 * li /2 * rho * g; %fixed (neutral buoyancy - hull with buoyancy engine half full/empty)
                        
                        % Bouyancy that varies with stroke
                        B_down = Bmax - pi* (di/2).^2 * li * rho * g; % (glider minimum buoyancy condition)
                        B_up = Bmax; % (glider maximum buoyancy condition)
                        
                        % Centre of bouyancy
                        %                         xb_neutral = [-(Li/2) + ( (pi/2) * 0.25 *li.^2 * ((Di/2).^2-(di/2).^2) + (Li/2 +0.5*li/2)*pi*(Li-0.5*li)*(Di/2).^2 ) ./ ( (pi * 0.5 * li ) * ((Di/2).^2-(di/2).^2) + pi*(Li-0.5*li)*(Di/2).^2   );0]
                        %                         xb_down = [-(Li/2) + ( (pi/2) * li.^2 * ((Di/2).^2-(di/2).^2) + (Li/2 + li/2)*pi*(Li-li)*(Di/2).^2 ) ./ ( (pi * li ) * ((Di/2).^2-(di/2).^2) + pi*(Li-li)*(Di/2).^2   );0]
                        %                         xb_up = [0; 0];%Centre of bouyancy at maximum buoyancy condition
                        
                        xb_neutral = [-(Li/2) + ( (li/2).^2 * ( Di.^2 - di.^2 ) + Di.^2 * (Li.^2 -(li/2).^2)  ) ./ ( 2*( Li * Di.^2 - (li/2) * di.^2) ) ; 0];
                        xb_down = [-(Li/2) + ( li.^2 * ( Di.^2 - di.^2) + Di.^2 * (Li.^2 -li.^2)  ) ./ ( 2* (Li * Di.^2 - li * di.^2) ) ; 0];
                        xb_up = [0; 0];%Centre of bouyancy at maximum buoyancy condition
                        
                        
                        % In the nuetral position and to achieve zero pitch, the weight force must be directly underneath the buoyancy force.
                        % to ensure stability (not calculated) the distance of the weight centroid is fixed at 10% of D
                        xMtotal_neutral  = xb_neutral + [0;-Di/5]; % Assumption addressed in paper
                        
                        
                        % xMtotal is known in the neutral position - its fixed lever arm xM can be calculated
                        xM = ( xMtotal_neutral * M_total - mi.* xm_neutral ) / M;
                        
                        % Centre of whole glider
                        xMtotal_down = (xm_down * mi + xM * M)/M_total;
                        xMtotal_up = (xm_up * mi + xM * M)/M_total;
                        
                        % Moment of Inertia
                        Inertia = M_total * (Di^2)/4 + M_total * (Li^2)/12; % assume a cyclinder around z axis at geometric centre
                        
                        C_b_down = xb_down; % position Cb when glider at maximum descent condition
                        C_m_down = xMtotal_down; % position Cm when glider in maximum descent condition
                        
                        C_b_up = xb_up; % position Cb when glider in maximum ascent condition - this will always be [0;0] by design
                        C_m_up = xMtotal_up; % position Cm when glider in maximum ascent condition
                        
                        % range of movement (in metres) of Cb and Cm from going fully up to fully down
                        C_b_movement = (C_b_down - C_b_up);
                        C_m_movement = (C_m_down - C_m_up);
                        
                        
                        %% Run model
                        
                        tspan = [0 tmax]; % time span of calculations
                        
                        odeparams = odeset('AbsTol',1e-3,'RelTol',1e-3);
                        IC = [0;0;0;0;0;0]; % Initial conditions - set to zero
                        % call RK solver - this is the dynamic model (ODE45
                        % found too stiff)
                        [t,z] = ode15s(@(t,z) glider_ode(...
                            t, z, t_act, C_b_up, C_b_down, C_b_movement, C_m_movement, C_m_up, ...
                            C_m_down, rho, g, M_total, Ai, xL, xD, Inertia, trigger_1, ...
                            tmax, li , Li, di, Di), tspan, IC, odeparams);
                        
                        switch combo
                            
                            case {3,4} % for unique solutions not over a range of parameters
                                
                                % direct outputs from solving the equations of motion
                                x = z(:,1); % x-position - horizontal distance travelled
                                vx = z(:,2); % x-velocity - horizontal velocity
                                y = z(:,3); % y-position - depth
                                vy = z(:,4); % y-velocity - vertical velocity
                                theta = z(:,5); % pitch angle
                                vtheta = z(:,6); % pitch angular velocity
                                
                                % derived outputs
                                phi = atan2(vy, vx); % glide angle
                                attack = wrapToPi(theta - phi); % angle of attack
                                
                                %if plotresults
                                figure(7)
                                plot(x,y)
                                subplot(2, 2, 1)
                                plot(x,y, 'LineWidth',2.0)
                                xlabel('Distance Travelled (m)')
                                ylabel('Depth (m)')
                                xlim([0 max(x)])
                                grid('on')
                                set(gca,'FontSize',18,'fontWeight','bold')
                                
                                
                                subplot(2, 2, 2)
                                plot(t,sqrt(vx.^2 + vy.^2), 'LineWidth',2.0)
                                xlabel('Time (s)')
                                ylabel('Velocity (m/s)')
                                xlim([0 tmax])
                                grid('on')
                                set(gca,'FontSize',18,'fontWeight','bold')                                
                                
                                %figure(4)
                                subplot(2, 2, [3,4])
                                plot(t,wrapToPi(theta)*180/pi, 'LineWidth',2.0)
                                hold on
                                plot(t,wrapTo180(attack*180/pi), 'LineWidth',2.0)
                                plot(t,wrapTo180(phi*180/pi), 'LineWidth',2.0)                                
                                plot(t([1 end]),[1 1]*90,'k--')
                                plot(t([1 end]),[1 1]*-90,'k--')
                                legend('Pitch','Attack', 'Glide')
                                xlabel('Time (s)')
                                ylabel('Angle (deg)')
                                xlim([0 tmax])
                                ylim([-100 100])
                                grid('on')
                                set(gca,'FontSize',18,'fontWeight','bold')
                                hold on
                                
                                
                                figure(8)
                                yyaxis left
                                plot(t,y, 'LineWidth',2.0)
                                xlabel('time (s)')
                                ylabel('Depth (m)')
                                xlim([0 max(t)])
                                grid('on')
                                set(gca,'FontSize',18,'fontWeight','bold')
                                yyaxis right
                                plot(t,wrapToPi(theta)*180/pi, 'LineWidth',2.0)
                                ylabel('Pitch (degrees)')
                                xlim([0 max(t)])
                                
                        end
                        
                        counter;
                        counter = counter + 1;
                        Percent_complete = 100*counter/( length(D) * length(L) * length(A) * length(d) * length(l) * length(m) )
                        
                        %% Save results
                        if saveresults
                            if combo == 1
                                txt = 'ext';
                            elseif combo == 2
                                txt = 'int';
                            else
                                txt = 'all';
                            end
                            
                            filename = [txt '_L' num2str(Li) '_D' num2str(Di) '_A' num2str(Ai) '_l' num2str(li) '_d' num2str(di) '_m' num2str(mi) '.mat'];
                            save(['Results' filesep filename],'z','t','Li','Di','Ai','li','di','mi', 'trigger_1', 'M_total', 'C_m_down', 'C_m_up','C_b_down', 'C_m_up', 'xD', 'xL','g','rho')
                        end
                    end
                end
            end
        end
    end
end

hold off
