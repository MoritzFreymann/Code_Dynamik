function [rob, dot_phi_vor] = AB2_dot_q( rob, j, dot_phi_vor )
% Die Funktion berechnet gemaess dem Adams-Bashforth-Verfahren zweiter
% Ordnung ausgehend von den Funktionswerten dot_Phi_i und dot_Phi_i-1 den
% Wert Phi_i+1 hier gleich rob.dot_q
% 
% Fuer den ersten Zeitschritt wird das explizites Euler-Verfahren verwendet
%
% j := aktueller Zeitschritt
% dot_phi_vor := dot_Phi am Zeitpunkt i-1
% rob := Struktur, in der Phi gespeichert wird

if j == 1            
    % Beim ersten Mal Werte ueber explizites Euler-Verfahren berechnen           
    rob.dot_q = rob.dot_q + rob.dt * rob.ddot_q;
else            
    % Gelenkgeschwindigkeit berechnen
    rob.dot_q = rob.dot_q + (rob.dt/2.0) * (3*rob.ddot_q - dot_phi_vor);
end

    % Speichere Beschleunigung
    dot_phi_vor = rob.ddot_q;
end

