function e_tau = Regelung( rob, j, Q, dot_Q )
% Berechnet den Fehler fuer den h-Vektor aller Kräfte ohne tau

% Merken von Gelenkwinkel/-geschwindigkeiten/-beschleunigungen
q = rob.q;
dot_q = rob.dot_q;
ddot_q = rob.ddot_q;

% Setzte ddot_q zu 0
rob.ddot_q = zeros(rob.N_Q,1);

% Berechne h_ist mit inverser Dynamik
rob = berechne_id(rob);
h_ist = rob.tau_id;

% Setze q_soll und dot_q_soll
rob.q = Q(:,j);
rob.dot_q = dot_Q(:,j);

% Berechne h_soll mit inverser Dynmaik
rob = berechne_id(rob);
h_soll = rob.tau_id;

e_tau = h_soll - h_ist;

% Zuruecksetzen von Gelenkwinkel/-geschwindigkeiten/-beschleunigungen
rob.q = q;    
rob.dot_q = dot_q;
rob.ddot_q = ddot_q;

end

