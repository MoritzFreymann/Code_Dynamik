function rob=berechne_bgl(rob)
%Berechnung der Bewegungsgleichungen des Systems (M, h und ddot_q)
%
% Alle notwendigen Funktionen werden von hier aus aufgerufen
%


%% 1. Berechnung des h-Vektors:
% Tipp: Nutzen Sie zur Berechnung von h die Funktion "berechne_id.m"

%Berechne inverse Dynamik tau(q, dot_q, ddot_q=0)
rob = berechne_id(rob, 'ddot_q=0' );
    
% Setze h gleich tau(q, dot_q, ddot_q=0)
rob.h = rob.tau_id;

%% 2. Berechnung der Massenmatrix
rob.M=zeros(rob.N_Q,rob.N_Q);

% Beitraege aller Koerper addieren
for i=1:length(rob.kl)
    % Anteil dieses Koerpers
   dM = rob.kl(i).m * rob.kl(i).Bi_Jt_o' * rob.kl(i).Bi_Jt_o ...
       + rob.kl(i).m * rob.kl(i).Bi_Jt_o' * tilde(rob.kl(i).Bi_r_s)' * rob.kl(i).Bi_Jr ...
       + rob.kl(i).m * rob.kl(i).Bi_Jr' * tilde(rob.kl(i).Bi_r_s)' * rob.kl(i).Bi_Jt_o ...
       + rob.kl(i).Bi_Jr' * rob.kl(i).I_o * rob.kl(i).Bi_Jr;

    %Anteil zur Gesamt-Massenmatrix addieren 
    rob.M=rob.M+dM;
end

% Die aktuellen Beschleunigungen berechnen
% Hier werden auch die Antriebsmomente der Regelung tau_reg beruecksichtigt

rob.ddot_q = rob.M \ ( rob.tau_reg - rob.h );
end

 
