clear; clc; close all;

AIR_DENSITY = 1.225;              % [kg/m^3]
AILERON_AREA = 2.9;               % [m^2]
AILERON_CHORD = 1.0;              % [m]
ACTUATOR_ARM_LENGTH = 0.427;      % [m]
HINGE_X_CHORD_RATIO = 0.7;
HINGE_Y_CHORD_RATIO = 0.02;

aileron_deflections_deg = -30:5:35;
angles_of_attack_deg = -10:2:15;
airspeeds_mps = linspace(70, 240, 20);

num_alphas = length(angles_of_attack_deg);
num_deflections = length(aileron_deflections_deg);

hinge_moment_coefficient_matrix = zeros(num_alphas, num_deflections);

disp('Evaluating XFOIL for all deflections...');

for i = 1:num_deflections
    % Prevent XFOIL panel singularity at exactly 0 degrees
    solver_deflection = aileron_deflections_deg(i);
    if solver_deflection == 0
        solver_deflection = 0.01;
    end
    
    fid = fopen('xfoil_in.txt', 'w');
    fprintf(fid, 'load NACA_64A210.txt\n');
    fprintf(fid, 'gdes\n');
    fprintf(fid, 'cadd\n\n\n\n');
    fprintf(fid, 'flap %f %f %f\n', HINGE_X_CHORD_RATIO, HINGE_Y_CHORD_RATIO, solver_deflection);
    fprintf(fid, 'exec\n\n');
    fprintf(fid, 'oper\n');
    fprintf(fid, 'iter 250\n');
    
    for j = 1:num_alphas
        fprintf(fid, 'alfa %d\n', angles_of_attack_deg(j));
        fprintf(fid, 'fmom %f %f\n', HINGE_X_CHORD_RATIO, HINGE_Y_CHORD_RATIO);
    end
    
    fprintf(fid, '\nquit\n');
    fclose(fid);
    
    system('xfoil.exe < xfoil_in.txt > xfoil_log.txt');
    
    log_text = fileread('xfoil_log.txt');
    matches = regexp(log_text, 'Hinge moment/span =\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)', 'tokens');
    
    parsed_coefficients = cellfun(@(x) str2double(x{1}), matches);
    
    % Ensure we only map values that actually converged to avoid size mismatch errors
    converged_length = min(num_alphas, length(parsed_coefficients));
    hinge_moment_coefficient_matrix(1:converged_length, i) = parsed_coefficients(1:converged_length);
end

disp('XFOIL evaluations complete. Computing moments and actuator forces...');

zero_alpha_index = find(angles_of_attack_deg == 0, 1);
ch_zero_alpha = hinge_moment_coefficient_matrix(zero_alpha_index, :);

% Fully vectorized actuator load math (Instantly computes the whole 2D grid)
constant_multiplier = (AIR_DENSITY * AILERON_AREA * AILERON_CHORD) / ACTUATOR_ARM_LENGTH;
deflection_vector = ch_zero_alpha(:) .* sind(aileron_deflections_deg(:)) * constant_multiplier;
velocity_vector = airspeeds_mps.^2;

actuator_load_matrix = deflection_vector * velocity_vector;

disp('Generating Simulink Lookup Table plots...');

figure(1);
surf(aileron_deflections_deg, angles_of_attack_deg, hinge_moment_coefficient_matrix);
xlabel('Aileron Deflection [deg]');
ylabel('Angle of Attack [deg]');
zlabel('Hinge Moment Coefficient (C_h)');
title('Simulink 2D Lookup Table: Hinge Moment Coefficient');

figure(2);
[v_mesh, delta_mesh] = meshgrid(airspeeds_mps, aileron_deflections_deg);
surf(v_mesh, delta_mesh, actuator_load_matrix);
xlabel('Airspeed [m/s]');
ylabel('Aileron Deflection [deg]');
zlabel('Actuator Load Q [N]');
title('Simulink 2D Lookup Table: Actuator Loads');