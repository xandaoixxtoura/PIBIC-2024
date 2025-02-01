%% tratamento dos sensores (antiga fun��o inicio)
    Ps_i = s_i;
    [ymaxA , xmaxA] = size(A);
    smax = janela*(ones(1,length(angs)));
    smax = smax + raio_robo;
    %s � um vetor cujos elementos representam o ponto da leitura m�xima (a priori) para
    %cada um dos sensores no sistema de coordenadas do rob�
    s = [smax.*cos(angs) ; smax.*sin(angs)]; % vetor no SC do rob� (ainda saturado)    
    Ri = [cos(Pos(3)) -sin(Pos(3)); sin(Pos(3)) cos(Pos(3))]; % matriz de rota��o
    %s_i � o vetor s colocado no sistema de coordenadas do ambiente (sem ru�do) (ainda saturado)
    s_i = Ri*s;
    s_i(1,:) = s_i(1,:) + Pos(1);
    s_i(2,:) = s_i(2,:) + Pos(2);
    
    pos_sensor = [raio_robo.*cos(angs) ; raio_robo.*sin(angs)];
    pos_sensor = Ri*pos_sensor;
    pos_sensor(1,:) = pos_sensor(1,:) + Pos(1);
    pos_sensor(2,:) = pos_sensor(2,:) + Pos(2);
    
    for k=1:length(angs)
        Vsx = round(linspace(pos_sensor(1,k),s_i(1,k),janela));
        Vsy = round(linspace(pos_sensor(2,k),s_i(2,k),janela));
        Vs = [Vsx ; Vsy];
        for j=1:janela
            if (Vs(1,j)>0 && Vs(1,j)<=xmaxA && Vs(2,j)>0 && Vs(2,j)<=ymaxA) % se a leitura esta dentro do mapa
                if A(Vs(2,j),Vs(1,j)) ~= 255
                    s_i(1,k) = Vs(1,j);
                    s_i(2,k) = Vs(2,j);
                    break;
                end
            end
        end
    end
    %retorna a medi��o para o sistema de coordenadas do rob�
    % O ROB� � ORIENTADO PARA O EIXO X.
    Ps_i(1,:) = s_i(1,:) - Pos(1);
    Ps_i(2,:) = s_i(2,:) - Pos(2);
    s = Ri\Ps_i;
    %adiciona ru�do na medi��o (a resolu��o da medi��o depende da resolu��o do
    %mapa)
    s = s + ruido*randn(2,length(angs));
    % sensor com ruido de medi��o no S.C. do ambiente
    aux = Ri*s;
    s2(1,:) = aux(1,:) + Pos(1);
    s2(2,:) = aux(2,:) + Pos(2);
    
    %% tratamento dos sensores (antiga fun��o FIM)
        
    v_sensor = sqrt( (s2(1,:)-pos_sensor(1,:)).^2 + (s2(2,:)-pos_sensor(2,:)).^2 ); 
    v_colidiu = sqrt( (s_i(1,:)-pos_sensor(1,:)).^2 + (s_i(2,:)-pos_sensor(2,:)).^2 ); % dist�ncia real entre o rob� e os obstaculos
    ds0 = sort(v_colidiu);  % ds0 � a dist�ncia do sensor sem ru�do com a menor leitura
    if ds0(1) <= 1 %colis�o de pior caso com 
        colidiu = 1;
    end