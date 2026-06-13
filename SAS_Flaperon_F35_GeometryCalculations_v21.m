clc; clear;
 
x_actuator_single = -20;  %min -20 max 22

X1 = x_actuator_single;
Y1 = 0;
X3 = 40.4;
Y3 = -71.0 ;
L1 = 109.3;
L2 = 42.7;
 

x_actuator = linspace(-20, 22, 40);
ailreon_deflection_angle = zeros(1, length(x_actuator));

syms x_2 y_2 real 

for i = 1:length(x_actuator)
    X1 =x_actuator(i);
    
    eq1 = (x_2 - X1)^2 + (y_2 - Y1)^2 == L1^2;
    eq2 = (X3 - x_2)^2 + (Y3 - y_2)^2 == L2^2;
    
    sol_loop = solve([eq1, eq2], [x_2, y_2]);
    
   
        x_opts = double(sol_loop.x_2);
        y_opts = double(sol_loop.y_2);
        
     
        x2_val = x_opts(2);
        y2_val = y_opts(2);
        ailreon_deflection_angle(i) = -atan2d(y2_val - Y3, x2_val -X3);
    end

% Plotting
figure;
plot(x_actuator, ailreon_deflection_angle, 'b-', 'LineWidth', 2);
grid on;
xlabel('Actuator Position (x\_actuator) [cm]');
ylabel('Aileron Angle (\beta) [degrees]');
title('\beta vs Actuator Position');