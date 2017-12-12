function e_tau = Regelung( rob, j, Q, dot_Q, Q_vd, dot_Q_vd )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    % Berechne h_ist mit inverser Dynamik
    rob = berechne_id(rob, 'ddot_q=0' );
    h_ist = rob.h;
    
    % Berechne h_soll mit inverser Dynmaik
    rob.q = Q(:,j);
    rob.dot_q = dot_Q(:,j);
    rob = berechne_id(rob, 'ddot_q=0' );
    h_soll = rob.h;
    
    e_tau = h_soll - h_ist;
    
    % Zurueckstzen 
    rob.q = Q_vd(:,j);
    rob.dot_q = dot_Q_vd(:,j);

end

