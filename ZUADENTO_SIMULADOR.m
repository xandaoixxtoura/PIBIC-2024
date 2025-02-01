    function ZUADENTO_SIMULADOR(experimento,robo_,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos, cont)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % inicialização das variáveis globais
    global  Pdes  Mapa Mapa2 i tempo tamos ;


    tamos = experimento.tamos; % tempo de amostragem da simualção em segundos
    for k = 1:nRobos
        robo_(k) = robo_(k).configuracao_inicial(experimento,tempo_max,k); 
    end
    %% carregando o mapa do ambiente
    A = imread(mapabmp);
    A = A(:,:,1);
    A = A./(max(max(A)));
    A = A.*255;
    % A = A(end:-1:1,:);
    %A = rgb2gray(A);
    Mapa2 = A;
    % A = A(end:-1:1,:); % utilizada no plot para parecer a imagem no SC padrão
    [Ay , Ax] = find(A~=255);
    Mapa = [Ax,Ay]';

    % Mapa a ser descoberto
    experimento.mapa_descoberto = 127*ones(size(Mapa2));


    % inicialização das variáveis do loop while
    tempo = 0:tamos:tempo_max;  % controle de tempo
    i = 0;  % contador

    colidiu = 0;
    concluiu = false;
    while  (~colidiu && (i*tamos<tempo_max) && ~concluiu)
          % distancia maior que 5 cm ou vlin maior q 5 cm/s ou vrot maior que 0.1 rad/s
        tic     
        % atualização das variáveis de controle de tempo
        i = i+1;          
        tempo(i+1) = i*tamos;   

        % Definição da flag que habilita a alocação
        experimento.flag = true;
        for k = 1:nRobos

            robo_(k) = robo_(k).simulacao_sensores(updateMapa2(A,robo_,nRobos,k));
            experimento.mapa_descoberto = robo_(k).DescobrimentoMapa(experimento.mapa_descoberto);
          
            tamos_controle = 0.01; %atualizar a cada 40 ms

            if mod(tempo(i),tamos_controle) == 0
                    robo_(k) = robo_(k).controle_e_navegacao(); 
            end


            %%%%%%%%%%%%%%%%%%%%% CONTROLADOR FIM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%% SIMULAÃ‡ÃƒO INÃ?CIO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if habilitaDinamica==1
                robo_(k) = robo_(k).simulacao(tamos,i);
            else
                robo_(k) = robo_(k).simulacao_apenas_cinematica(tamos,i);
            end

             if robo_(k).colidiu
                robo_(k) = robo_(k).simulacao_falha();
                colidiu = 1;
             end

             experimento.flag = experimento.flag&robo_(k).chegou;
        end
         if (i == 1)
            checks = [];
        end



        alocacao_tarefas_4



        % PLOT DO GRÁFICO "ON LINE"
        tamos_plot = 0.1; %atualizar a cada 100 ms
        if mod(tempo(i),tamos_plot) == 0
            if habilitaPlot, plot_graficos_online; end
        end 
        i*tamos
      %  if (i*tamos == 600*tamos)
         %   maps = experimento.mapa_descoberto ; 
         %   save -mat test2.mat maps experimento

      %  end
         check = sum(sum(experimento.mapa_descoberto == 255));
          check_porcent = sum(sum(experimento.mapa_descoberto == 255))/sum(sum(Mapa2 == 255))
          checks = [checks; check_porcent];
          if (check_porcent >= 0.94 )
             concluiu = true; 
          end
         end

    tempo = tempo(1:i);
    for k = 1:nRobos
        robo_(k) = robo_(k).vecCorrecao(i,habilitaDinamica);
    end

    %%%% Salvando os dados no arquivo
     save -mat exp_16_artg.mat experimento robo_ tempo A Ax Ay Pdes habilitaDinamica  checks
         end