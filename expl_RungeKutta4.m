function rob = expl_RungeKutta4( rob, j, phi_i_vor )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Merke Anfangswerte fuer q, dot_q und ddot_q
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

% Setzte dot_q auf Ursprungswert zurueck
rob.dot_q = phi_i;

% Berechne q_i+1 ueber Adams-Bashforth-Verfahren 2. Ordnung
[rob, dot_q_vor] = AB2_q( rob, j, phi_i_vor );

% Berechne dot_q_i+1 = phi_i+1
rob.dot_q = phi_i + rob.dt * ( (1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4 );

% Setzte ddot_q auf Ursprungswert zurueck
rob.ddot_q = dot_phi_i;

end

