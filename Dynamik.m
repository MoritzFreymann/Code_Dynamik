%Aufraeumen
close all;
clear all;
clc

% Fuege Pfad hinzu
addpath('C:\Users\Moritz\My\Studium\Roboterdynamik_Praktikum\1_Direkte_Kinematik\Code_Direkte_Kinematik\Direkte_Kinematik');

%% 1. Parameter und Strukturen initialisieren

%Lade Sollbahn im Arbeitsraum in Workspace
load( 'Gelenkwinkelbahn.mat' );

%Erzeuge Roboterstruktur
rob = erstelle_roboter();

%Datenmatrix fuer Viewer - Sollbahn
V = zeros(3,4,6,length(T));

% 
DGLVerfahren = 'AB2/AM3'; % Verfahren zur Loesung der DGL
                        % 'Euler_ex'...explizites Euler-Verfahren
                        % 'AB2/AM3' ...Adams-Bashforth-Verfahren 2. Ordnung
                        %           ... und und Adams-Moulton-Verfahren 3. Ordnung
                        % 'ex_RK4'  ...explizites (klassisches)
                        %           ...Runge-Kutta-Verfahten 4. Ordnung
                        
dot_q_vor = zeros(rob.N_Q,1);
dot_q_vvor = zeros( size(dot_q_vor) );
ddot_q_vor = zeros( size(dot_q_vor) );

% Zwangskraefte
for i=1:rob.N_Q
    F = zeros(3,1,6,length(T));
    M = zeros(3,1,6,length(T));
end

%% Inverse Dynamik

%Variable zur Speicherung der berechneten Gelenkmomente
Tau_id = zeros( size(Q) );

%Berechne Gelenkmomentenverlauf fuer gegebene Bahn
for i=1:length(T)
    
    %Setze Gelenkwinkel und Zeit
    rob.q = Q(:,i);
    rob.dot_q = dot_Q(:,i);
    rob.ddot_q = ddot_Q(:,i);
    
    rob.zeit = T(i);
 
    % Berechne inverse Dynamik
    rob = berechne_id(rob);
    
    % Gelenkmomente zur spaeteren Analyse speichern
    Tau_id(:,i) = rob.tau_id;   
    
    % Speichere Vektoren B0_r_i und Transformationsmatrizen A_i0 fuer Visualisierung im Viewer
    for l = 1:6
        V( :, 1, l, i ) = rob.kl(l).B0_r_i;
        V( :, 2:4, l, i ) = rob.kl(l).A_i0;
    end
end

%Drehmomentenverlauf plotten
figure();
plot( T, Tau_id(1,:), ...
      T, Tau_id(2,:), ...
      T, Tau_id(3,:), ...
      T, Tau_id(4,:), ...
      T, Tau_id(5,:), ...
      T, Tau_id(6,:) );
  
h=legend( '$\tau_1$','$\tau_2$','$\tau_3$','$\tau_4$','$\tau_5$','$\tau_6$');
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$\tau$ in [Nm]','Interpreter','latex');

%Winkelgeschwindigkeit plotten
figure();
plot( T, dot_Q(1,:), ...
      T, dot_Q(2,:), ...
      T, dot_Q(3,:), ...
      T, dot_Q(4,:), ...
      T, dot_Q(5,:), ...
      T, dot_Q(6,:) );

h=legend( '$\dot q_1$','$\dot q_2$','$\dot q_3$','$\dot q_4$','$\dot q_5$','$\dot q_6$' );
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$\dot{q}$ in [rad/s]','Interpreter','latex');

k = 1;
K = k*eye(rob.N_Q);
%% Direkte Dynamik

%Setze Roboter auf Anfangsposition
rob.q = Q(:,1);
rob.dot_q = dot_Q(:,1);
rob.ddot_q = ddot_Q(:,1);

%Variablen zur Speicherung der berechneten Bahn
Q_vd = zeros( size(Tau_id) );
dot_Q_vd = zeros( size(Tau_id) );
ddot_Q_vd = zeros( size(Tau_id) );

e_tau = zeros( size(Tau_id) );

% Datenmatrix fuer Viewer - Istbahn
V_vd = zeros(3,4,rob.N_Q,length(T));

% Berechne Bahn aus Drehmomenten der inversen Dynamik
for j=1:length(T)
    
%     % Regelung
    e_tau(:,j) = Regelung( rob, j, Q, dot_Q );
    
    % Setze Antriebsmoment
    rob.tau_reg = Tau_id(:,j) - K*e_tau(:,j);
       
    % Setze die aktuelle Zeit
    rob.zeit = T(j);
    
    % Berechne Winkelbeschleunigungen (direkte Dynamik)
    rob = berechne_bgl(rob);
    
    % Gelenkwinkelbahn abspeichern
    Q_vd(:,j) = rob.q;
    dot_Q_vd(:,j) = rob.dot_q;
    ddot_Q_vd(:,j) = rob.ddot_q;
    
    % Speichere Vektoren B0_r_i und Transformationsmatrizen A_i0 fuer Visualisierung im Viewer
    for l = 1:6
        V_vd( :, 1, l, j ) = rob.kl(l).B0_r_i;
        V_vd( :, 2:4, l, j ) = rob.kl(l).A_i0;
    end
    
    % Beschleunigungen und Geschwindigkeiten integrieren,
    % um Gelenkwinkel/-geschwindigkeiten fuer nachsten Zeitschritt zu
    % berechnen
    
    if strcmp(DGLVerfahren,'Euler_ex') == true        
        % Gelenkgeschwindigkeiten/-winkel ueber explizites Euler-Verfahren berechnen
        rob.q = rob.q + rob.dt * rob.dot_q;
        rob.dot_q = rob.dot_q + rob.dt * rob.ddot_q;        
        
    elseif strcmp(DGLVerfahren,'AB2/AM3') == true
        % Verwende Adams-Bashforth-Verfahren 2. Ordnung
        % und Adams-Moulton-Verfahren 3. Ordnung
        
        % Berechne Gelenkgeschwindigkeiten mit Adams-Bashforth
        [rob, ddot_q_vor] = AB2_dot_q( rob, j, ddot_q_vor ); 

        % Berechne Gelenkwinkel mit Adams-Moulton
        [rob, dot_q_vor, dot_q_vvor] = AM3_q( rob, j, dot_q_vor, dot_q_vvor );

    elseif strcmp(DGLVerfahren, 'ex_RK4') == true
        % Benutze expliziztes (klassiches) Runge-Kutta-Verfahren 4.Ordnung
        
        % Berechne Gelenkwinkel/-geschwindigkeiten
        [rob, dot_q_vor] = expl_RungeKutta4( rob, j, dot_q_vor );
        
    else
        % Ungueltige Option
        error('Ungueltige Option gewaehlt!')
    end   
    
    % Berechne Zwangskraefte
    for i=rob.N_Q:-1:1
        if i~=rob.N_Q
            % Berechne Kraefte
            F(:,:,i,j) = rob.kl(i).m * rob.kl(i).Bi_ddot_r_s...
            - rob.kl(i).m * rob.kl(i).A_i0 * rob.B0_g...
            - F(:,:,i+1,j);
            % Berechne Momente
            M(:,:,i,j) = rob.kl(i).I_o * rob.kl(i).Bi_dot_omega +...
            tilde(rob.kl(i).Bi_omega) * rob.kl(i).I_o * rob.kl(i).Bi_omega +...
            rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).Bi_ddot_r_i -...
            rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).A_i0 * rob.B0_g...
            - M(:,:,i+1,j);
        else
            % Berechne Kraefte
            F(:,:,i,j) = rob.kl(i).m * rob.kl(i).Bi_ddot_r_s...
            - rob.kl(i).m * rob.kl(i).A_i0 * rob.B0_g;

            % Berechne Momente
            M(:,:,i,j) = rob.kl(i).I_o * rob.kl(i).Bi_dot_omega +...
            tilde(rob.kl(i).Bi_omega) * rob.kl(i).I_o * rob.kl(i).Bi_omega +...
            rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).Bi_ddot_r_i -...
            rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).A_i0 * rob.B0_g;
        end
    end
    
end

% Speichere die Gelenkwinkel fuer den Viewer in .csv-Datei
write_data(T,V,6,'trajectory_Dynamik_Soll.csv');
write_data(T,V_vd,6,'trajectory_Dynamik_Ist.csv');

% Differenzen der Winkel zur Ueberpruefung plotten
e_Q = Q - Q_vd;

number = sprintf('%d', k); 
title = strcat('k = ',number);

figure('Name',title,'NumberTitle','off');
plot( T, e_Q(1,:), ...
      T, e_Q(2,:), ...
      T, e_Q(3,:), ...
      T, e_Q(4,:), ...
      T, e_Q(5,:), ...
      T, e_Q(6,:) );
  
h=legend( '$e_{q_1}$','$e_{q_2}$','$e_{q_3}$','$e_{q_4}$','$e_{q_5}$','$e_{q_6}$','Location','northwest');
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$e_q$ in [rad]','Interpreter','latex');

%% Plotte Zwangskraefte
for i=1:rob.N_Q
    
    number = sprintf('%d', i); 
    title = strcat('Gelenk ',number);

    figure('Name',title,'NumberTitle','off');
    
    % Berechne 2D-Matrix aus 4D-Matrix
    f = reshape ( F(:,1,i,:), 3, length(T) );
    m = reshape ( M(:,1,i,:), 3, length(T) );
    
    % Plotte Kraefte
    subplot(2,1,1);
    plot(   T, f(1,:), ...
            T, f(2,:), ...
            T, f(3,:)   );

    h=legend( '${F_x}$','${F_y}$','${F_z}$','Location','northwest');
    h.Interpreter='latex';
    xlabel( 't in [s]','Interpreter','latex');
    ylabel( 'Kraefte in [N]','Interpreter','latex');

    % Plotte Momente
    subplot(2,1,2);
    plot(   T, m(1,:), ...
            T, m(2,:), ...
            T, m(3,:)   );
        
    h=legend( '${M_x}$','${M_y}$','${M_z}$','Location','northwest');
    h.Interpreter='latex';
    xlabel( 't in [s]','Interpreter','latex');
    ylabel( 'Momente in [Nm]','Interpreter','latex');

end
