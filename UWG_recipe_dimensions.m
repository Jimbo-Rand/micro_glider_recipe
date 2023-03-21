function [t_act, Vol_glider, Vol_engine, C_b_up, C_b_down, C_b_movement, C_m_movement, C_m_up, C_m_down, rho, g, M_glider, Area_wetted, Area_wing, C_p_wing, C_p_drag, I, trigger_1, M_ballast_total, X_ballast_fitted_to_hull, X_b_down, X_m_down, X_b_up, X_m_up] = UWG_recipe_dimensions(g, rho, t_act, trigger_1, M_hull, M_act, D, L, d, l, E_o, Y_act_ballast, Y_hull_ballast, l_o, balance_hull_act_neutral, ballast_split, C_p_wing, Wing_length_total);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% put into seperate function
%% Conventions
%  _hull means hull, sensors and controls - DOES NOT INCLUDE actuator or ballast
%  _act means actuator
%  _bal means ballast
%  _glider means the whole thing ready to deploy

%  Glider local dimensional reference point is centre line, on the edge of the hull, buoyancy engine end.
%  This is used to establish dimensions prior to being converted to the
%  global model reference point at the glider geometric centre
%  M is mass, , L is length, d and D is diameter, Vol is volume
%  ie M_hull_act means the mass of glider hull, sensors, controls and actuator
%  x and y are orientation/axis
%  splits and shares are given as percentages measured from the buoyancy engine end.


% MODEL REFERENCE POINT IS GEOMETRIC MID POINT OF GLIDER [0; 0]; % frame of reference - geometric centre of glider
% C_m centre of mass, C_b centre of buoyancy, C_p centre of pressure
% X is distance (lever arm)
% +ve rotation is anti-clockwise
% x and y displacements are graph axis convention (travelling north and east is +ve, travelling south and west is -ve)

% conditions are physical and performance limits where the glider is
% impossible or cannot operate. They are accounted for in the main script.
% Conditions in this scrip mearly generate a text warning in the command
% window

% condition
if d > D
disp('buoyancy engine too big for hull')
end

% derived volumes
Vol_glider = pi * L * (D/2)^2; % Glider volume
Vol_engine = pi * l * (d/2).^2; % Total engine volume
Vol_engine_neutral = Vol_engine * (l_o/100); % engine volume at neutral buoyancy position

% locations (from glider geometric centre) where glider without ballast balances (as this can be physically determined if necessay)
X_hull_act_neutral = L * (balance_hull_act_neutral/100 - 1/2); % X axis position where mass glider at neutral position without ballast acts
X_act_neutral = (E_o + l * l_o/100)-(L/2); % X axis position where actuator at neutral mass acts
X_hull = (X_hull_act_neutral * (M_hull + M_act) - X_act_neutral * M_act)/M_hull; % X axis position where mass hull and fixed equipments acts

% how much ballast and where fitted in what split ?
M_ballast_total = rho * (Vol_glider - Vol_engine_neutral) - M_hull - M_act; % total amount of ballast required by glider to work

M_ballast_fitted_to_actuator = (ballast_split/100)* M_ballast_total; % mass of actuator ballast
M_ballast_fitted_to_hull = M_ballast_total - M_ballast_fitted_to_actuator;  % mass of hull ballast
M_glider = M_hull + M_act + M_ballast_total; % mass of glider

% condition
if M_glider < M_hull
  disp('hull too heavy - fantasy negative ballast required') 
end

% location of hull fitted ballast to achieve neutral pitch with actuator fitted ballast on the actuator
% moment of hull ballast balances moment of actuator ballast
X_ballast_fitted_to_hull = - (M_ballast_fitted_to_actuator * X_act_neutral)/M_ballast_fitted_to_hull; % X axis position where mass of hull, fixed equipments and ballast acts

% condition
if X_ballast_fitted_to_hull >= abs(L/2)
    disp('model failure - ballast located outside of hull')
end

% Location of actuator (and its ballast) going up and down
X_act_up = (E_o) - L/2;
X_act_down = (E_o +  l) - L/2;

% Calculate locations where mass and buoyancy acts upon the complete glider
X_m_up = (X_hull * M_hull + X_act_up * (M_act + M_ballast_fitted_to_actuator) + X_ballast_fitted_to_hull * M_ballast_fitted_to_hull)/M_glider;
X_m_down = (X_hull * M_hull + X_act_down * (M_act + M_ballast_fitted_to_actuator) + X_ballast_fitted_to_hull * M_ballast_fitted_to_hull)/M_glider;
Y_m = (Y_act_ballast * M_ballast_fitted_to_actuator + Y_hull_ballast* M_ballast_fitted_to_hull)/M_ballast_total;

X_b_up = 0; % Cb when glider going up - symetry determines at the [0,0] position
X_b_down = L/2 -(Vol_glider - Vol_engine)/(pi * (D/2)^2)/2;

% condition - maybe only applies when in buoyancy engin at full or empty ????
if X_b_down > X_m_down
%     disp('model failure going down - centre of buoyancy forward of centre of mass')
end
if X_b_up < X_m_up
    disp('model failure going up - centre of buoyancy aft of centre of mass')
end

%% Wing


Wing_chord = Wing_length_total/10; % This mimics typical commercial glider wings

% Fin
%not included yet

% tempory solution to vary wing area - scales wing and so determines which drag and lift curve used to calculate Cl and Cd
% wing_flag = 2; % 1 = short, 2 = baseline, 3 = long
% % wing scale factor - determine which wing fitted
% if wing_flag == 1
%     k = 0.5;
% elseif wing_flag == 2
%     k = 1;
% else
%     k = 2;
% end

% the no messing simple assumption version
Area_wing = Wing_chord * Wing_length_total;
Area_hull =  pi*(D^2)/4; 
Area_wetted = Area_wing + Area_hull;

% Area_wing = Wing_chord * Wing_length_total * k * sin(abs(attack)); % treated as a flat plate - good enough ???
% Area_hull =  cos(attack) * pi*(d_glider^2)/4 + sin(abs(attack)) * (pi*d_glider*L_glider)/2 ; % % silhouette area of glider, m^2, depends on angle of attack as to area visible



%% Other bits

X_p = L/3 - L/2; % centre of pressure - assume fixed at the moment (YES - BUT should check that this is true using CFD) - check how sensitive to parameter
I = M_glider * (D^2)/4 + M_glider * (L^2)/12; % Moment of Inertia - assume a cyclinder around z axis at geometric centre

%% Centres in vector form

C_b_up = [X_b_up; 0]; % position Cb when ascending - this should always be [0;0] 
C_b_down = [X_b_down; 0]; % position Cb when ascending

C_m_up = [X_m_up; Y_m]; % position Cm when ascending
C_m_down = [X_m_down; Y_m]; % position Cm when ascending

C_p_drag = [X_p; 0]; % Cp - fixed at -0.145 m on the centreline - will move in reality - refinement for a later day



% range of movement of Cb and Cm going fully up and fully down
C_b_movement = C_b_down - C_b_up;
C_m_movement = C_m_down - C_m_up;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end