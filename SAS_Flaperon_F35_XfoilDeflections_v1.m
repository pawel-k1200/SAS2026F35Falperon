clear; clc; close all;

% --- 1. Parameters ---
rho = 1.225;        % Air density [kg/m^3]
S = 2.9;            % Flaperon surface area [m^2]
c = 1.0;            % Mean aerodynamic chord [m]
L1 = 0.427;         % Control horn lever arm [m]
x_hinge = 0.7;      % Hinge X position (XFOIL)
y_hinge = 0.02;     % Hinge Y position (XFOIL)

% --- 2. Flight Envelopes ---
delta = -30:5:35;             % Deflection angles [deg]
alpha = -10:2:15;             % Angles of attack [deg]
V = linspace(70, 240, 20);    % Airspeeds [m/s]

n_alpha = length(alpha);
n_delta = length(delta);
n_V = length(V);

% --- 3. Run XFOIL to get Hinge Moment Coefficients (Ch) ---
Ch = zeros(n_alpha, n_delta);

for i = 1:n_delta
    d = delta(i);
    if d == 0
        d = 0.01; % Prevent exact zero to avoid XFOIL division errors
    end
    
    % Write XFOIL instructions
    fid = fopen('xfoil_in.txt', 'w');
    fprintf(fid, 'load NACA_64A210.txt\n');
    fprintf(fid, 'gdes\n');
    fprintf(fid, 'cadd\n\n\n\n');
    fprintf(fid, 'flap %f %f %f\n', x_hinge, y_hinge, d);
    fprintf(fid, 'exec\n\n');
    fprintf(fid, 'oper\n');
    fprintf(fid, 'iter 250\n');
    
    for j = 1:n_alpha
        fprintf(fid, 'alfa %d\n', alpha(j));
        fprintf(fid, 'fmom %f %f\n', x_hinge, y_hinge);
    end
    
    fprintf(fid, '\nquit\n');
    fclose(fid);
    
    % Execute XFOIL and read results
    system('xfoil.exe < xfoil_in.txt > xfoil_log.txt');
    log_str = fileread('xfoil_log.txt');
    matches = regexp(log_str, 'Hinge moment/span =\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)', 'tokens');
    parsed_coeffs = cellfun(@(x) str2double(x{1}), matches);
    
    % Store data safely
    n_converged = min(n_alpha, length(parsed_coeffs));
    Ch(1:n_converged, i) = parsed_coeffs(1:n_converged);
end

% Extract Ch values strictly for alpha = 0 (which is index 6 in the alpha array)
Ch_alpha0 = Ch(6, :);

% --- 4. Calculate Final Actuator Loads (Q) ---
Q = zeros(n_V, n_delta);

for i = 1:n_V
    for j = 1:n_delta
        
        % 1. Dynamic Pressure
        q = 0.5 * rho * V(i)^2;
        
        % 2. Aerodynamic Torque (Moment)
        M = q * S * c * Ch_alpha0(j);
        
        % 3. Effective Mechanical Lever Arm
        theta = 90 - delta(j); 
        lever = L1 * sind(theta);
        
        % 4. Final Actuator Load
        Q(i, j) = M / lever;
        
    end
end

% --- 5. Plotting Results ---
figure(1);
surf(delta, alpha, Ch);
xlabel('Flaperon Deflection [deg]');
ylabel('Angle of Attack [deg]');
zlabel('Ch Coefficient');
title('Hinge Moment Coefficient (XFOIL)');

figure(2);
surf(delta, V, Q);
xlabel('Flaperon Deflection [deg]');
ylabel('Airspeed [m/s]');
zlabel('Load on actuator[N]');
title('Actuator Load');