function e_tau = Regelung( rob, j, Q, dot_Q )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    ddot_q = rob.ddot_q;
    rob.ddot_q = zeros(rob.N_Q,1);
    % Berechne h_ist mit inverser Dynamik
    rob = berechne_id(rob, 'ddot_q~=0' );
    h_ist = rob.tau_id;
    
    rob.ddot_q = ddot_q;
   
    % Merke q_ist und dot_q_ist
    q = rob.q;
    dot_q = rob.dot_q;
    
    % Setze q_soll und dot_q_soll
    rob.q = Q(:,j);
    rob.dot_q = dot_Q(:,j);
    
    % Berechne h_soll mit inverser Dynmaik
    rob = berechne_id(rob, 'ddot_q=0' );
    h_soll = rob.tau_id;
    
    e_tau = h_soll - h_ist;
    
    % Zuruecksetzen von q_ist und dot_q_ist
    rob.q = q;
    
    rob.dot_q = dot_q;

end

