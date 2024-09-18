function [dz] = glider_ode( t, z, t_act, C_b_up, C_b_down, C_b_movement, C_m_movement, C_m_up, C_m_down, rho, g, M_total, Ai, xL, xD, Inertia, trigger_1, tmax, li , Li, di, Di)

%% simultaneous second order differentials for glider model
% output vector z has the 6 differential outputs
% units: meters, seconds, Newtons, kg, radians

% derived values from the 6 outputs
phi = atan2( z(4), z(2) ); % glide angle
attack = wrapToPi( z(5) - phi ); % angle of attack

% derived volumes
Vol_glider = pi * Li * ( Di/2 )^2; % Glider volume
Vol_engine = pi * li * ( di/2 )^2; % Total engine volume

% Resolved velocity of glider
Vel = sqrt( z(4)^2 + z(2)^2 );

[Cd, Cl] = liftdrag(attack, Vel); % obtain Cd and Cl using angle of attack data
%% Rotation matrix

R = [cos(z(5)) -sin(z(5)); sin(z(5)) cos(z(5))];

%% Diving and ascending

% TIME BASED DIVE CYCLE

if  t <= t_act/2 % buoyancy engine from half empty to full on surface
    Vol = ( Vol_glider - 0.5 * Vol_engine ) - 0.5 * Vol_engine *( t/( t_act/2 ) ); % Glider volume and half full buoyancy engine at start -  minus the changing volume of the remaining half of the buoyancy engine at time t
    %     Vol = Vol_glider - Vol_engine*(t/t_act);
    buoy_arm = R * ( C_b_up + ( t/( t_act/2 ) ) * C_b_movement ); % how arms change with changing volume
    mass_arm = R * ( C_m_up + ( t/( t_act/2 ) ) * C_m_movement );
elseif t > t_act/2 && t <= trigger_1
    Vol = Vol_glider - Vol_engine; % Buoyancy engine full of water - in maximum descent configuration
    buoy_arm = R * C_b_down;
    mass_arm = R * C_m_down;
elseif  t > trigger_1 && t <= trigger_1 + t_act % glider at depth switching buoyancy engine from full to empty
    Vol = Vol_glider - Vol_engine * (1 - ( t-trigger_1 )/t_act );
    buoy_arm = R * ( C_b_down - ( ( t-trigger_1 )/t_act ) * C_b_movement );
    mass_arm = R * ( C_m_down - ( ( t-trigger_1 )/t_act ) * C_m_movement );
else
    Vol = Vol_glider; % Buoyancy engine empty - in maximum ascent configuration
    buoy_arm = R * C_b_up;
    mass_arm = R * C_m_up;
end

% Buoyancy, mass, drag and lift forces
F_B = rho * Vol * g; % Buoyancy Force
F_M = M_total * g; % Force due to mass

% shadow_area of glider presented to fluid flow
shadow_area = abs( ( pi * ( di/2 )^2 + ( 0.1 * Ai ) ) * cos(phi) + ( ( Li * Di ) + (Ai ) ) * sin(phi) );     %(frontal area wings  = 10% wing area)

% drag and lift forces as modified by angle of attack
F_D = 0.5 * rho * Cd  * shadow_area * Vel^2; % Drag Force
F_L = 0.5 * rho * Cl * shadow_area * Vel^2; % Lift Force

%% solve odes

dz = zeros(6,1); % initialize output space
dz(1) = z(2); % z(1) is the differential of x, which is x_dot
dz(2) = ( -F_L * sin(phi) - F_D * cos(phi) )/M_total;
dz(3) = z(4); % same idea as z(1), but for y
dz(4) = ( F_B - F_M + F_L * cos(phi) - F_D * sin(phi) )/M_total;

% Rotate the centres according to pitch
lift_arm = R * xL;
drag_arm = R * xD;

% Torque from mass
T_mass_all = cross( [mass_arm; 0], [0; -1; 0] ) * F_M;
T_mass = T_mass_all(3);

% Torque from buoyancy
T_buoy_all = cross( [buoy_arm; 0], [0; 1; 0] ) * F_B;
T_buoy = T_buoy_all(3);

% Torque from drag
T_drag_all = cross( [drag_arm; 0], -[cos(phi); sin(phi); 0] ) * F_D;
T_drag = T_drag_all(3);

% Torque from lift
T_lift_all = cross( [lift_arm; 0], [sin(phi); cos(phi); 0] ) * F_L;
T_lift = T_lift_all(3);

% Resistance to pitching of hull (water resistance)
model_experiment_damping_factor = 1.0; % scaling factor to increase model damping based on experimental results
T_rotate = -1/160 * rho * Di * Li^5 * z(6)^2 * model_experiment_damping_factor;

dz(5) = z(6); % same idea as z(1), but for theta

dz(6) = ( T_buoy + T_mass + T_drag + T_lift + T_rotate )./Inertia;

%lift_matrix = (F_M,  F_B, F_D, F_L)
end