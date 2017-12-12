function [ rob, phi_i_vor, phi_i_vvor ] = AM3_q( rob, j, phi_i_vor, phi_i_vvor )
% rob := Struktur, in der Gelenkwinkel/-geschwindigkeiten/-beschleunigungen
% gespeicher werden
% j := aktueller Zeitschritt
% phi_i_vor := Phi zum Zeitpunkt i (hier gleich rob.dot_q)
% phi_i_vvor := Phi zum Zeitpunkt i (hier gleich rob.dot_q)

if j == 1
    % Benutze Trapez-Regel (AM2)
    rob.q = rob.q + rob.dt/2.0 * (phi_i_vor + rob.dot_q);
else
    % Benutze AM3
    rob.q = rob.q + rob.dt/12.0 * ( 5*rob.dot_q + 8*phi_i_vor - phi_i_vvor );
end


end

