clear; clc;

% 1. Define Grids
deltas = -30:5:35;
alphas = -10:2:15;
Ch_matrix = zeros(length(alphas), length(deltas));
Cl_matrix = zeros(length(alphas), length(deltas));
Cd_matrix = zeros(length(alphas), length(deltas));

for i = 1:length(deltas)
    
    current_delta = deltas(i);
    if current_delta == 0
        current_delta = 0.01; 
    end
    
    fid = fopen('xfoil_in.txt', 'w');
    fprintf(fid, 'load NACA_64A210.txt\n');
    fprintf(fid, 'gdes\n');
    fprintf(fid, 'cadd\n\n\n\n');
    fprintf(fid, 'flap 0.7 0.02 %f\n', current_delta); % Changed to %f for decimals
    fprintf(fid, 'exec\n\n'); 
    
    fprintf(fid, 'oper\n');
    fprintf(fid, 'iter 250\n');
    
    
    for j = 1:length(alphas)
        fprintf(fid, 'alfa %d\n', alphas(j));
        fprintf(fid, 'fmom 0.7 0.02\n'); 
    end
    
    fprintf(fid, '\nquit\n');
    fclose(fid);
        
    
    disp(['Running XFOIL for delta = ', num2str(deltas(i)), '...']);
    system('xfoil.exe < xfoil_in.txt > xfoil_log.txt');
        
    
    log_text = fileread('xfoil_log.txt');
    
    matches_ch = regexp(log_text, 'Hinge moment/span =\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)', 'tokens');
    
    if ~isempty(matches_ch)
        ch_vals = cellfun(@(x) str2double(x{1}), matches_ch);
        ch_vals = ch_vals(:);
        
        min_len = min(length(alphas), length(ch_vals));
        Ch_matrix(1:min_len, i) = ch_vals(1:min_len);
    else
        disp(['  -> Solver failed completely for delta = ', num2str(deltas(i))]);
    end
    
end

disp('All Runs Complete. Generating Plot...');

Ch_used=Ch_matrix(6,:);
V=linspace(70,240,20);
for i = 1:length(Ch_used)
    for j = 1:length(V)
        Q(i, j)=Ch_used(i)*V(j)^2*1.225*2.9*1/0.427*sind(deltas(i));
    end
end


figure(1);
surf(deltas, alphas, Ch_matrix);
axis auto;
xlabel('Aileron Deflection \delta (deg)'); 
ylabel('Angle of Attack \alpha (deg)'); 
zlabel('Hinge Moment Coefficient (C_h)');
title('Simulink 2-D Lookup Table Data');
figure(2);
surf(V, deltas, Q);
axis auto;
xlabel('Airspeed (m/s)'); 
ylabel('Aileron Deflection \delta (deg)'); 
zlabel('Q (N)');
title('Simulink 2-D Lookup Table Data');