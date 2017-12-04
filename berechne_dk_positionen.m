function rob = berechne_dk_positionen_vektorkette(rob)
    % Berechnung der Positions-Groessen der direkten Kinematik (Lage und Orientierung)
    % ueber die Bildung einer Vektorkette

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % Ersetzen Sie die nachfolgenden Rueckgaben durch Ihre Berechnungen
    % ---------------------------------------------------------------------
    
    % Berechnung Koerper 1
        % Drehmatrix vom Vorgaenger zum i-ten Koerper (i_A_i-1)
        rob.kl(1).A_iv = Az( rob.q(1) );
        
        % Drehmatrix vom B0-KOS ins Bi-KOS: 
        rob.kl(1).A_i0 = rob.kl(1).A_iv;
        
        % Position des Ursprungs des i-ten Koerpers im Bi-KOS:
        rob.kl(1).Bi_r_i = zeros(3,1);

        % Position des Ursprungs im B0-System (fuer Animation benoetigt):
        rob.kl(1).B0_r_i = zeros(3,1);
    
    % Berechnung fuer alle Koerper
    for i = 2:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;

        % Relativkinematik: Position und Orientierung relativ zum
        % Vorgaenger
        % ------------------------------------------------------------------
        % Verschiebungsvektor vom Vorgaenger zum Koerper i im KOS des
        % Vorgaengers (Bv)
        rob.kl(i).Bv_r_vi = [                      rob.kl(i).a;...
                             -sin(rob.kl(i).alpha)*rob.kl(i).d;...
                              cos(rob.kl(i).alpha)*rob.kl(i).d];

        % Drehmatrix vom Vorgaenger zum i-ten Koerper 
        rob.kl(i).A_iv = Az( rob.q(i) ) * Ax( rob.kl(i).alpha );    % i_A_v       
   
        % Absolute Position und Orientierung
        % ----------------------------------
        
        % Drehmatrix vom B0-KOS ins Bi-KOS: 
        rob.kl(i).A_i0 = rob.kl(i).A_iv * rob.kl(vor).A_i0; % i_A_0 = i_A_v * v_A_0

        % Position des Ursprungs des i-ten Koerpers im Bi-KOS:
        rob.kl(i).Bi_r_i = rob.kl(i).A_iv * ( rob.kl(vor).Bi_r_i + rob.kl(i).Bv_r_vi ); % i_r_i = i_A_v (v_r_v + v_r_vi)

        % Position des Ursprungs im B0-System (fuer Animation benoetigt):
        rob.kl(i).B0_r_i = (rob.kl(i).A_i0)' * rob.kl(i).Bi_r_i;    % 0_r_0 = i_A_0^T * i_r_i
    end

    % Position des TCP im B0-System berechnen
    rob.w = rob.kl(rob.N_Q).B0_r_i + (rob.kl(rob.N_Q).A_i0)' * rob.BN_r_N_tcp;  % 0_r_TCP = 0_r_N + N_A_6^T * N_r_N,TCP
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end
