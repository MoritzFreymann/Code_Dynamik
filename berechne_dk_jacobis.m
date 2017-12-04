function rob = berechne_dk_jacobis(rob,Verfahren)
    % Berechnung der Jacobi-Matrizen der Rotation, Translation und Arbeitsraumkoordinaten
    % Hinweis: vor dem Aufruf dieser Funktion muss berechne_dk_positionen ausgefuehrt werden, um
    % Zugriff auf aktuellen Drehmatrizen zu haben. Dies ist bereits in Inverse_Kinematik.m
    % implementiert.

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % Ersetzen Sie die nachfolgenden Rueckgaben durch Ihre Berechnungen
    % ---------------------------------------------------------------------
    
    % Berechnung fuer Koerper 1
        i = 1;
        % relative Jacobi-Matrix der Rotation des Koerpers
        % -----------------------------------------
        Bi_Jr_rel = zeros(3,rob.N_Q);
        Bi_Jr_rel(3,i) = 1;
        % ...(Formel 2.3.33, S.18 Skript)
        
        % Jacobi-Matrix der Rotation des Koerpers
        % -----------------------------------------
        rob.kl(i).Bi_Jr = Bi_Jr_rel;
        % allgemein:    i_J_r_i = i_A_v * v_J_v + i_J_rel_i ...( Formel 2.3.25, S.17 Skript)
        % hier:         v_J_v = 0 (Vorgaenger ist Inertialsystem)       

        % Jacobi-Matrix der Translation des Koerpers
        % --------------------------------------------       
        rob.kl(i).Bi_Jt_o = zeros(3,rob.N_Q);
        % allgemein:    i_J_t_i = i_A_v * ( v_J_t_v + tilde_v_r_vi^T * v_J_r_v ) ...( nach Formel 2.3.26, S.17 Skript & Formel 2.3.34, S.18 Skript)
        % hier:         v_J_t_v = 0 (Vorgaenger ist Inertialsystem);
        %               v_r_tilde_vi = 0 (Ursprung des KOS identisch)
        %               -> i_J_t_i = 0;
        
    % Berechnung fuer alle Koerper
    for i=2:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;

        % relative Jacobi-Matrix der Rotation des Koerpers i
        % -----------------------------------------
        Bi_Jr_rel = zeros(3,rob.N_Q);
        Bi_Jr_rel(3,i) = 1;
        % ...(Formel 2.3.33, S.18 Skript)
        
        % Jacobi-Matrix der Rotation des Koerpers i
        % -----------------------------------------
        rob.kl(i).Bi_Jr = rob.kl(i).A_iv * rob.kl(vor).Bi_Jr + Bi_Jr_rel;
        % i_J_r_i = i_A_v * v_J_v + i_J_r_rel_i ...( Formel 2.3.25, S.17 Skript)

        % Jacobi-Matrix der Translation des Koerpers i
        % --------------------------------------------       
        rob.kl(i).Bi_Jt_o = rob.kl(i).A_iv * ( rob.kl(vor).Bi_Jt_o + ( tilde(rob.kl(i).Bv_r_vi) )' * rob.kl(vor).Bi_Jr );
        % i_J_t_i = i_A_v * ( v_J_t_v + tilde_v_r_vi^T * v_J_r_v ) ...( nach Formel 2.3.26, S.17 Skript & Formel 2.3.34, S.18 Skript)
    end

    % Jacobi-Matrizen fuer TCP
    % ------------------------
    % Jacobi-Matrix der Rotation des TCP dargestellt im B0-KOS
    B0_Jr = rob.kl(rob.N_Q).A_i0' * rob.kl(rob.N_Q).Bi_Jr;
    %  = N_Q_A_0^T * N_Q_J_r_N_Q

    % Jacobi-Matrix der Translation des TCP dargestellt im B0-KOS
    B0_Jt_o = rob.kl(rob.N_Q).A_i0' * ( rob.kl(rob.N_Q).Bi_Jt_o + ( tilde(rob.BN_r_N_tcp) )' *  rob.kl(rob.N_Q).Bi_Jr );
    % allgemein:    0_J_t_i = i_A_0^T * i_A_v * ( v_J_t_v + tilde_v_r_vi^T * v_J_r_v ) ...( nach Formel 2.3.26, S.17 Skript & Formel 2.3.34, S.18 Skript)
    % hier:         i_A_0^T = tcp_A_0^T = n_q_A_0^T (KOS gleich)
    %               i_A_v = tcp_A_n_q = E (KOS gleich)

    % Jacobi-Matrix der Arbeitsraum-Koordinaten
    % -----------------------------------------
    rob.Jw = B0_Jt_o;

    %% --- ENDE ARBEITSBEREICH --------------------------------------------

    % Platzhalter (noch nicht zu bearbeiten)
    if strcmp(Verfahren,'xxx') == true
    rob.Jw = [ B0_Jt_o; B0_Jr];
    end
end