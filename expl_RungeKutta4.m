function [rob, phi_i_vor] = expl_RungeKutta4( rob, j, phi_i_vor )
% Die Funktion berechnet  ausgehend von den Werten fuer die
% Gelenkwinkel/-geschwindigkeiten/-beschleunigungen zum Zeitpunkt i
% die Gelenkwinkel/-geschwindigkeiten zum Zeitpunkt i+1
%
% Fuer die Berechnung der Gelenkwikel wird entweder das 
% Adams-Bashforth-Verfahren 2. Ordnung oder das implizizte Euler-Verfahren 
% verwendet 
% 
% Fuer die Berechnung der Gelenkwikelgeschwindigkeiten wird das explizite
% (klasische) Runge-Kutta-Verfahren 2. Ordnung verwendet
%
% rob := Struktur, in der Gelenkwinkel/-geschwindigkeiten/-beschleunigungen
% gespeicher werden
% j := aktueller Zeitschritt
% phi_i_vor := Phi zum Zeitpunkt i (hier gleich rob.dot_q)

%% Merke Anfangswerte fuer q, dot_q und ddot_q
dot_phi_i = rob.ddot_q;
phi_i = rob.dot_q;  
q_i = rob.q;

%% k1
% Berechne k1
k1 = rob.ddot_q;

%% k2
% Berechne q und dot_q fuer naechsten Punkt
rob.q = q_i + rob.dt/2.0 * phi_i;
rob.dot_q = phi_i + rob.dt/2.0 * k1;
% Berechne k2 ueber BGL
rob = berechne_bgl(rob);
k2 = rob.ddot_q;

%% k3
% Berechne dot_q fuer naechsten Punkt
rob.dot_q = phi_i + rob.dt/2.0 * k2;
% Berechne k3 ueber BGL
rob = berechne_bgl(rob);
k3 = rob.ddot_q;

%% k4
% Berechne q und dot_q fuer naechsten Punkt
rob.q = q_i + rob.dt * phi_i;
rob.dot_q = phi_i + rob.dt * k3;
% Berechne k4 ueber BGL
rob = berechne_bgl(rob);
k4 = rob.ddot_q;

%% Berechne endgueltige Werte

% Setzte q und dot_q auf Ursprungswert zurueck
rob.q = q_i;
rob.dot_q = phi_i;

% Berechne q_i+1 ueber Adams-Bashforth-Verfahren 2. Ordnung
[rob, phi_i_vor] = AB2_q( rob, j, phi_i_vor );

% Berechne dot_q_i+1 = phi_i+1
rob.dot_q = phi_i + rob.dt * ( (1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4 );


% Setzte ddot_q auf Ursprungswert zurueck
rob.ddot_q = dot_phi_i;

end

