%% Analysis of the circuit of the pressure sensor
%  Author: Julien Biral, Master's Thesis at CNMAT, UC Berkeley
%  Date: 05-11-2014
%  
%  ========================================================================
%  DESCRIPTION:
%  

%% DC Analysis of the sensor circuit
%  This analysis is done to maximize the dynamic of the circuit. In DC, the
%  capacitor C can be replaced by an open-circuit. Maximizing dynamics is
%  thus achieved by choosing the adequate value of the resistor R.
%  Special trick: this analysis uses the star-delta transform to simplify
%  the circuit.

% Clear the workspace of unwated elements
close all;
clear all;
clc;

% =========================== %
% Initialization of variables %
% =========================== %

% Number of points to compute:
N = 10000;

% ADC resolution
nbits = 10;

% Power Voltage of the circuit:
% Set to 1 to normalize the value computed during the analysis.
Vd = 1;

% Boundaries of the variable resistor:
RvL = 20e3;
RvH = 1e5;

% Variable resistor array:
% Only the minimum and the maximum values are considered here as the aim is
% to maximize the dynamic of the circuit. Intermediate values are not
% useful for this purpose.
R1 = [RvL, RvH];
R2 = [RvL, RvH];
R3 = [RvL, RvH];
R4 = [RvL, RvH];

% Resistor sweep:
% The sweep is done to calculte the R value which gives the best dynamic
R = 1e5/N : 1e5/N : 1e5;

% Initialization of the voltages arrays:
V0 = zeros(length(R1), length(R));
V1 = zeros(length(R1), length(R));
V2 = zeros(length(R1), length(R));
V3 = zeros(length(R1), length(R));
V4 = zeros(length(R1), length(R));

% =============================== %
% Operating points of the circuit %
% =============================== %

for i = 1:length(R1)
    
    for j = 1:length(R)
        
        % Kennelly's Star-Delta Transformation:
        Rab = ( R2(i)*R3(i) + R(j)*(R2(i)+R3(i)) ) / R(j);
        Rbc = ( R2(i)*R3(i) + R(j)*(R2(i)+R3(i)) ) / R2(i);
        Rac = ( R2(i)*R3(i) + R(j)*(R2(i)+R3(i)) ) / R3(i);
        
        % Kennelly's Delta-Star Transformation:
        Rat = ( Rab * R(j) ) / ( Rab + 2*R(j) );
        Rbt = Rat;
        Rdt = ( R(j)^2 ) / ( Rab + 2*R(j) );
        
        % Bottom parralel resistors:
        Rp1 = ( Rat * (R(j) + R1(i) + Rac) + Rac * (R(j) + R1(i)) ) ...
            / (R(j) + R1(i) + Rac);
        Rp2 = ( Rbt * (R(j) + R4(i) + Rbc) + Rbc * (R(j) + R4(i)) ) ...
            / (R(j) + R4(i) + Rbc);
        
        % Equivalent resistor of the circuit:
        Rtot = ( Rdt * (Rp1 + Rp2) + Rp1*Rp2 ) / (Rp1 + Rp2);
        
        % Currents:
        Idt = Vd / Rtot;
        Ita = Rp2 / (Rp1 + Rp2) * Idt;
        Itb = Rp1 / (Rp1 + Rp2) * Idt;
        I1  = Rac / (R(j) + R1(i) + Rac) * Ita;
        I4  = Rbc / (R(j) + R4(i) + Rbc) * Itb;
        
        % Output voltages:
        V0(i,j) =  R(j) * I1;
        V1(i,j) = (R(j) + R1(i)) * I1;
        V3(i,j) = (R(j) + R4(i)) * I4;
        V4(i,j) =  R(j) * I4;
        V2(i,j) =  R(j) * (V1(i,j)*R3(i) + V3(i,j)*R2(i)) ...
            / (R2(i)*R3(i) + R(j)*R2(i) + R(j)*R3(i));
    end
    
end

% Initialization of the text output:
fprintf(strcat('Pressure Sensor - DC Analysis:\n', ...
    '==============================\n'));
fprintf('Parameters:\n-----------\nRvLow = %i, RvHigh = %i, Vd = %i\n',...
    RvL, RvH, Vd);

fprintf('\nAnalysis:\n---------\n');

% ======================== %
% Maximization of dynamics %
% ======================== %

% Plot of the output voltages and computation of maximum dynamics points:
set(0,'DefaultAxesFontName', 'Latin Modern')
set(0,'DefaultAxesFontSize', 10)
figure;

% Choose a ratio that zoom or dezoom the plots
ratio = .05;

% V0
subplot(3, 1, 1);
plot(R, V0(1,:), '-r');
hold on;
plot(R, V0(2,:), '-b');
[resolution, ind] = max( abs(V0(1,:) - V0(2,:)) );
line([R(ind), R(ind)], ...
    [min(V0(1,ind), V0(2,ind)), max(V0(1,ind), V0(2,ind))], ...
    'LineStyle', '-.', 'LineWidth', 2, 'Color', [0.4, 0.8, 0.4]);
hold off;

xlim([R(1), R(length(R))]);
ylim([(1-ratio)*min(min(V0(1,:)), min(V0(2,:))), ...
    (1+ratio)*max(max(V0(1,:)), max(V0(2,:)))]);

title('V0 over R', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('R (Ohm)');
ylabel('Magnitude');

fprintf('V0  - Max dynamic for R = %i and resolution of %i\n', R(ind), ...
    floor(resolution*2^nbits/Vd));


% V1 - Vd
Vd1 = Vd - V1;
subplot(3, 1, 2);
plot(R, Vd1(1,:), '-r');
hold on;
plot(R, Vd1(2,:), '-b');
[resolution, ind] = max( abs(Vd1(1,:) - Vd1(2,:)) );
line([R(ind), R(ind)], ...
    [min(Vd1(1,ind), Vd1(2,ind)), max(Vd1(1,ind), Vd1(2,ind))], ...
    'LineStyle', '-.', 'LineWidth', 2, 'Color', [0.4, 0.8, 0.4]);
hold off;

xlim([R(1), R(length(R))]);
ylim([(1-ratio)*min(min(Vd1(1,:)), min(Vd1(2,:))), ...
    (1+ratio)*max(max(Vd1(1,:)), max(Vd1(2,:)))]);

title('Vd1 over R', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('R (Ohm)');
ylabel('Magnitude');

fprintf('Vd1 - Max dynamic for R = %i and resolution of %i\n', R(ind), ...
    floor(resolution*2^nbits/Vd));


% V2
subplot(3, 1, 3);
plot(R, V2(1,:), '-r');
hold on;
plot(R, V2(2,:), '-b');
[resolution, ind] = max( abs(V2(1,:) - V2(2,:)) );
line([R(ind), R(ind)], ...
    [min(V2(1,ind), V2(2,ind)), max(V2(1,ind), V2(2,ind))], ...
    'LineStyle', '-.', 'LineWidth', 2, 'Color', [0.4, 0.8, 0.4]);
hold off;

xlim([R(1), R(length(R))]);
ylim([(1-ratio)*min(min(V2(1,:)), min(V2(2,:))), ...
    (1+ratio)*max(max(V2(1,:)), max(V2(2,:)))]);

title('V2 over R', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('R (Ohm)');
ylabel('Magnitude');

fprintf('V2  - Max dynamic for R = %i and resolution of %i\n', R(ind), ...
    floor(resolution*2^nbits/Vd));


% Best R that maximizes the voltage difference between the variable
% resistors:
% V1 - V0
V10 = V1 - V0;
figure;
subplot(2, 1, 1);
plot(R, V10(1,:), '-r');
hold on;
plot(R, V10(2,:), '-b');
[resolution, ind] = max( abs(V10(1,:) - V10(2,:)) );
line([R(ind), R(ind)], ...
    [min(V10(1,ind), V10(2,ind)), max(V10(1,ind), V10(2,ind))], ...
    'LineStyle', '-.', 'LineWidth', 2, 'Color', [0.4, 0.8, 0.4]);
hold off;

xlim([R(1), R(length(R))]);
ylim([(1-ratio)*min(min(V10(1,:)), min(V10(2,:))), ...
    (1+ratio)*max(max(V10(1,:)), max(V10(2,:)))]);

title('V10 over R', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('R (Ohm)');
ylabel('Magnitude');

fprintf('V10 - Max dynamic for R = %i and resolution of %i\n', R(ind), ...
    floor(resolution*2^nbits/Vd));

% V1 - V0
V12 = V1 - V2;
subplot(2, 1, 2);
plot(R, V12(1,:), '-r');
hold on;
plot(R, V12(2,:), '-b');
[resolution, ind] = max( abs(V12(1,:) - V12(2,:)) );
line([R(ind), R(ind)], ...
    [min(V12(1,ind), V12(2,ind)), max(V12(1,ind), V12(2,ind))], ...
    'LineStyle', '-.', 'LineWidth', 2, 'Color', [0.4, 0.8, 0.4]);
hold off;

xlim([R(1), R(length(R))]);
ylim([(1-ratio)*min(min(V12(1,:)), min(V12(2,:))), ...
    (1+ratio)*max(max(V12(1,:)), max(V12(2,:)))]);

title('V12 over R', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('R (Ohm)');
ylabel('Magnitude');

fprintf('V12 - Max dynamic for R = %i and resolution of %i\n', R(ind), ...
    floor(resolution*2^nbits/Vd));


fprintf('------------------------------\n');

% Computes the best R value to have the max total dynamic in the circuit
% The '2*' factor in front of V0 and V1 is there to take into account the
% symmetry of V0 and V1 with V4 and V3 respectively.
% Normalization of each outputs:
V0norm  = abs( (V0(1,:) - V0(2,:)) / (max(V0(1,:)) - min(V0(2,:))) );
Vd1norm = abs( (Vd1(1,:) - Vd1(2,:)) / (max(Vd1(1,:)) - min(Vd1(2,:))) );
V2norm  = abs( (V2(1,:) - V2(2,:)) / (max(V2(1,:)) - min(V2(2,:))) );
V10norm = abs( (V10(1,:) - V10(2,:)) / (max(V10(1,:)) - min(V10(2,:))) );
V12norm = abs( (V12(1,:) - V12(2,:)) / (max(V12(1,:)) - min(V12(2,:))) );
Vw      = 2 * V0norm + 2 * Vd1norm + V2norm + 2 * V10norm + 2 * V12norm;

% Plot the voltages weighted sum
figure;
plot(R, Vw, '-r');
[~, ind] = max( Vw );
hold on;
plot(R(ind), Vw(ind), 'o', 'LineWidth', 2, 'Color', [0.4, 0.8, 0.4]);
line([R(ind), R(ind)], [min(Vw), Vw(ind)], ...
    'LineStyle', '-.', 'LineWidth', 2, 'Color', [0.4, 0.8, 0.4]);

xlim([R(1), R(length(R))]);
ylim([(1-ratio)*min(Vw), (1+ratio)*max(Vw)]);

title('Vw over R', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('R (Ohm)');
ylabel('Magnitude');

% Computation of the best R:
fprintf('---> Max dynamic of the circuit for R = %i\n', R(ind));

% ========================= %
% Clearing of the workspace %
% ========================= %

clear I1 I4 Idt Ita Itb N R R1 R2 R3 R4 Rab Rac Rat Rbc Rbt Rdt Rp1 Rp2 ...
    Rtot RvH RvL V0 V0norm V1 V1norm V2 V2norm V3 V4 Vd Vw i ind j ratio;

%% AC Analysis of the sensor circuit
%  The first part of the AC Analysis is to determine the impedance matrix
%  of each two-port network of the circuit. Due to its symmetry, there is
%  two two-port networks to consider: the first one is the low-pass filter
%  part of the sensor and the second one is the low-pass at the input of
%  the ADC converter.

% Clear the workspace of unwated elements
close all;
clear all;
clc;

% =========================== %
% Initialization of variables %
% =========================== %

% Number of points to compute:
N = 10000;

% Boundaries of the variable resistor:
RvL = 8e3;
RvH = 1e5;

% Variable resistor array:
% Only the minimum and the maximum values are considered here as the aim is
% to maximize the dynamic of the circuit. Intermediate values are not
% useful for this purpose.
R1 = [RvL, RvH];

% Resistor:
% The ideal value that maximizes dynamics can be determined with the DC
% analysis.
R = 10000;

% Capacitor:
% The capacitor need to be chosen to ensure a minimum cutoff frequency of
% 500 Hz. As the boundary if Rv tends to infinite is the cutoff frequency
% F= 1/(2*pi*RC), C is determined in this way.
Fs = 1000;
C = 1/(2*pi*R*Fs/2);

% Elements of the circuit of the ADC input:
% As it is impossible to know the exact capacitor and resistor at the input
% of the ADC, the worst case is taken into account in these computations.
% The following values correspond to [Typical value, Max value], regarding
% to the ADC datasheet, in 10-bits mode.
% Ra = [2e3, 5e3];
% Ca = [4e-12, 5e-12];

Ra = 2e3;
Ca = 4e-12;

% =================== %
% Transmission Matrix %
% =================== %

% In this section, the two different transmission matrix of the circuit are
% implemented for further calculation. The transmission matrix is useful to
% compute the transfer function of two-ports networks in cascade.

for i = 1:length(R1)
    
    clear Ts Ta;
    % Transmission matrix of the low-pass two-ports network of the pressure
    % sensor:
    Ts = [
            tf([R*R1(i)*C, R+R1(i)], R),    tf([R*R1(i)*C, R1(i)], [R*C, 1]);
            tf([R*C, R+R1(i)], R),          tf(1)
         ];

    % Transmission matrix of the input of the ADC converter
    Ta = [
            tf([Ra*Ca, 1], 1),    tf(Ra);
            tf([Ca, 0], 1),       tf(1)
         ];
     
    % ================== %
    % Transfer Functions %
    % ================== %

    % If a output sees a variation in an adjacent resistor
    % Transmission Matrix = LP*ADC

    % If a output sees a variation in non adjacent resistor
    % Transmission Matrix = LP*LP*ADC
    % And so on.
    clear Tm;
    Tm = Ts;

    % Once the transmission matrix have been found, the transfer function of
    % each case can easily be computed.
    clear Tf;
    Tf = [tf(1), tf(1), tf(1), tf(1)];

    for j = 1:max(size(Tf))

        Tf(j) = 1 / (Tm(1,1) );
        %minreal(Tf(j));
        fprintf('Bandwidth through %i low-pass filters for Rv = %i is %i\n', j, R1(i), bandwidth(Tf(j)));
        Tm = Tm*Ts;

    end
   
end

%% Zout
% Number of points to compute:
N = 10000;


% Boundaries of the variable resistor:
RvL = 8e3;
RvH = 1e5;

% Variable resistor array:
% Only the minimum and the maximum values are considered here as the aim is
% to maximize the dynamic of the circuit. Intermediate values are not
% useful for this purpose.
R1 = [RvL, RvH];
R2 = R1;
R3 = R1;
R4 = R1;

% Resistor sweep:
% The sweep is done to calculte the R value which gives the best dynamic
R = 1e5/N : 1e5/N : 1e5;

for i=1:length(R1)
    for j = 1:length(R)
        Rth(i,j) = R(j) - R(j)^2 * ( R(j)+ 1/(1/(R1(i)+R(j)) + 1/R2(i)));
    end
end

plot(R, Rth(1,:), 'r');
hold on;
plot(R, Rth(2,:), 'b');