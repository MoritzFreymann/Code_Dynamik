function rob = berechne_id(rob)
%Inverse Dynamik fuer Roboter rob berechnen
% Die Ergebnisse werden wiederum in der Struktur rob. gespeichert
%
% Im einzelnen werden Impuls- und Drallaenderung aller Koerper berechnet und
% ueber die Jacobi-Matrizen in die zwangsfreien Richtungen projiziert.
% Im Ergebnis wird die "linke Seite der Bewegungsgleichung"
% M*ddot_q + h berechnet und in tau_id gespeichert
% Es werden alle noetigen Groessen hier berechnet

%1. Mit Null initialisieren
rob.tau_id=zeros(rob.N_Q,1);

%2. Kinematik berechnen
rob=berechne_dk_positionen(rob);
rob=berechne_dk_geschwindigkeiten(rob);
rob=berechne_dk_beschleunigungen(rob);
rob=berechne_dk_jacobis(rob,'xxx');

%3. Berechnung fuer alle Koerper: Impuls- und Drallaenderung
for i=1:length(rob.kl)
    
    % Absolutbeschleunigung des Schwerpunkts:
    rob.kl(i).Bi_ddot_r_s = rob.kl(i).Bi_ddot_r_i + 0; %...
        %( tilde(rob.kl(i).Bi_dot_omega) + tilde(rob.kl(i).Bi_omega) * tilde(rob.kl(i).Bi_omega) ) *...
        %rob.kl(i).Bi_r_s;
    % ---------------------------------------------------------------------
    % allg.: i_a_si = i_a_i +...
    % ( i_dot_tilde_omega + i_tilde_omega * i_tilde_omega) * i_r_i,si +...
    %   2* i_tilde_omega * dot_i_r_i,si + ddot_i_r_i,si
    % ...(2.3.19) Skript Seite 16
    % hier: i_r_i,si = const. => dot_i_r_i,si = ddot_i_r_i,si = 0
    %  =>   i_a_si = i_a_i +...
    % ( i_dot_tilde_omega + i_tilde_omega * i_tilde_omega) * i_r_i,si
    % ---------------------------------------------------------------------
    
    % Impulsaenderung - Schwerkraft
    F = rob.kl(i).m * rob.kl(i).Bi_ddot_r_s - rob.kl(i).m * rob.B0_g;
    %...nach (3.3.21) Skript Seite 36
    
    % Drallaenderung - Moment der Schwerkraft
    T = rob.kl(i).I_o * rob.kl(i).Bi_dot_omega +...
        tilde(rob.kl(i).Bi_omega) * rob.kl(i).I_o * rob.kl(i).Bi_omega +...
        rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).Bi_ddot_r_s -...
        rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.B0_g;
    % I0_i * dot_omega_i +  tilde_omega_i * I0_i * omega_i +...
    % m_i * tilde_r_i,si * a_i - M0_g
    %...nach (3.3.21) Skript Seite 36
    
    % Projektion auf zwangsfreie Richtungen und Addition zu tau_id
    rob.tau_id = rob.tau_id + [rob.kl(i).Bi_Jt_o; rob.kl(i).Bi_Jr]' * [F; T];
    
end
end



