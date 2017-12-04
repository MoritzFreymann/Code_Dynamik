function rob = berechne_dk_geschwindigkeiten(rob)
    % Berechnung der Geschwindigkeits-Groessen der direkten Kinematik

    %% --- ARBEITSBEREICH: ------------------------------------------------
    % Ergaenzen Sie bitte hier die fehlenden Stellen
    % Ersetzen Sie die nachfolgenden Rueckgaben durch Ihre Berechnungen
    % ---------------------------------------------------------------------
    % Berechnung fuer Koerper 1
    i = 1;
    % Relative Winkelgeschwindigkeit berechnen
        % ----------------------------------------
        rob.kl(i).Bi_omega_rel = Ax( -rob.kl(i).alpha) * [ 0; 0; rob.dot_q(i) ];    % = v_omega_v,i

        % Absolute Winkelgeschwindigkeit berechnen
        % ----------------------------------------
        rob.kl(i).Bi_omega = rob.kl(i).A_iv; % relative Winkelgeschwindigkeit = absolut fuer Koerper 1

        % Absolute Translationsgeschwindigkeit berechnen
        % ----------------------------------------------
        rob.kl(i).Bi_dot_r_i = 0;   % auschliesslich Rotation
    % Berechnung fuer restliche Koerper
    for i = 2:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;

        % Relative Winkelgeschwindigkeit berechnen
        % ----------------------------------------
        rob.kl(i).Bi_omega_rel = Ax( -rob.kl(i).alpha) * [ 0; 0; rob.dot_q(i) ];    % = v_omega_v,i

        % Absolute Winkelgeschwindigkeit berechnen
        % ----------------------------------------
        rob.kl(i).Bi_omega = rob.kl(i).A_iv * ( rob.kl(vor).Bi_omega + rob.kl(i).Bi_omega_rel ); % i_omega_i = i_A_v * ( v_omega_v + v_omega_v,i )

        % Absolute Translationsgeschwindigkeit berechnen
        % ----------------------------------------------
        % allg: i_v_i = i_A_v * ( v_v_v + v_tilde_omega * v_r_v,i + v_dot_r_v,i )
        % hier: v_dot_r_v,i = 0
        % ----------------------------------------------
        rob.kl(i).Bi_dot_r_i = rob.kl(i).A_iv * ( rob.kl(vor).Bi_dot_r_i + tilde(rob.kl(vor).Bi_omega) * rob.kl(i).Bv_r_vi );
    end

    % Geschwindigkeit des TCP im B0-System berechnen
    rob.dot_w = rob.kl(rob.N_Q).A_i0' * ( rob.kl(rob.N_Q).Bi_dot_r_i + tilde(rob.kl(rob.N_Q).Bi_omega) * rob.BN_r_N_tcp );
    %% --- ENDE ARBEITSBEREICH --------------------------------------------
end
