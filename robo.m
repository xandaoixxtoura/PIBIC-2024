classdef robo
    % robo classe que representa o robô
    
    properties
    % Características físicas do robô    
        r  % 5.3000
        L  % 5.6250
        Modelo  % [2×2 ss]
        raio  % 8
        Modcin  % [2×2 double]
        ganhoft  % [2×1 double]
        Vmax  % 34.1570
        Wmax  % 6.0724
        ruido  %  1
        saturacao  % 200
    % Informações sensoriais do robô
        id
        Pos 
        hPos
        Pdes
        v_sensor
        s
        s_i
        s2 
        angs
        sensorObstaculo
        sensorRobos
        colidiu
        falhou
        chegou
    % Variáveis Controlador baixo nível PI
        err_i  % integral do erro
    % Variáveis Controle desvio de obstáculo 
        Freal  % memória
    % Sinais de Controle
        U    % Contem o sinal de controle da roda esquerda e direita
        Ud   % Sinal de controle desejado
        X    % estado dinâmico do sistema
        Ksi_r
    % Informações para o plot
        plotInfo
        plt
        colors
        
        
    end
    
    methods
        function obj = robo(id)
            %%% Parâmetros de configuração das características do robô.
            % Wd/FId = A/(s+B); %velocidade angula na saída em rad/s
            %    -15.66
            %   --------
            %   s + 4.87
            %    -15.11
            %   ---------
            %   s + 5.014
            % We/FIe = C/(s+D); %velocidade angula na saída em rad/s
            %    17.75
            %   ---------
            %   s + 5.833
            %    13.11
            %   ---------
            %   s + 4.286
            obj.r = 5.3; %raio da roda
            obj.L = 11.25/2; %distância da roda ao centro do robô (metade da distancia entre as rodas)
            k_w_to_fi = 2*obj.L/obj.r;
            A = 15.11;
            A_ = A*k_w_to_fi;
            B = 5.014;
            C = -13.11;
            C_ = C*(-k_w_to_fi);
            D = 4.286;
            Ma = [-B 0 ; 0 -D];
            Mb = [A_ 0 ; 0 C_];
            Mc = [obj.r/2 , obj.r/2 ; obj.r/(2*obj.L) -obj.r/(2*obj.L)];
            Md = [0 0 ; 0 0];

            obj.Modelo = ss(Ma,Mb,Mc,Md);
            
            % U = [U_d ; U_e] entradas de -1 a 1
            % X = [FI_d ; FI_e] estado
            % Y = [V ; W] saídas

            obj.raio = 8; %raio do robô para verificar colisão e plotar o robô [cm]
            obj.Modcin = Mc; %modelo cinemático direto
            obj.ganhoft = [B/A_ ; D/C_];
            % U = inv(obj.Modcin)*[V ; W];

            fi = [1 ; 1]./[obj.ganhoft];
            aux = obj.Modcin*(fi);
            obj.Vmax = aux(1); %cálculo da vellcidade linear máxima a partir do modelo identificado
            fi = [1 ; -1]./[obj.ganhoft];
            aux = obj.Modcin*(fi);
            obj.Wmax = aux(2); %cálculo da vellcidade angular máxima a partir do modelo identificado
            
            obj.ruido = 0; %desvio padrão do ruído do sensor
            obj.saturacao = 60; %limite do alcance sensorial em cm
            obj.id = id;
                        
            obj.err_i = [0;0];
            obj.Freal = [0;0];
            obj.Ksi_r = [0;0;0];
            
            obj.colors = ['b','g','r','c','m','y','b','g','r'];
        end
        
        function obj = controle_e_navegacao(obj)
                   controlador2            
        end      
             
        function [Ksir_real, X] = dinamica_zuadento(obj,Tamos)
               
                U = [obj.U obj.U obj.U]; %[Ud ; Ue]
            % % %     U = U'; % descomentar essa linha pra usar no octave
                %Xp = Ma*X + Mb*U
                % Y = Mc*X + Md*U
                [Y,T,X] = lsim(obj.Modelo,U ,[0:Tamos:2*Tamos],obj.X);
                X = X(2,:)';
                Ksir_real = [Y(2,1) ; 0 ; Y(2,2)];
        end
        
        function obj = simulacao(obj,tamos,iteracao)
            
            if ~obj.falhou
                
                R = [cos(obj.Pos(3)) sin(obj.Pos(3)) 0 ; -sin(obj.Pos(3)) cos(obj.Pos(3)) 0 ; 0 0 1]; % matriz de rota??o
                [Ksir_real, X] = dinamica_zuadento(obj,tamos); %retorna as velocidades V e W reais no SC do rob?
                obj.X = X;
                Ksi_I = R\Ksir_real; % coloca as velocidades V e W reais no SC do ambiente
                obj.Pos = obj.Pos + Ksi_I*tamos; % atualiza??o da posi??o do rob? (integra??o no SC do ambiente)
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = Ksir_real(1);
                Wmedido = Ksir_real(3);

                FIreal = obj.Modcin\[Vmedido ; Wmedido];
                Ureal = [obj.ganhoft].*FIreal;

                fi = obj.Modcin\[obj.Ksi_r(1) ; obj.Ksi_r(3)]; %FI = [FI_d ; FI_e];
                Ud = [obj.ganhoft].*fi;
                Ksir_d = [obj.Ksi_r(1) ; obj.Ksi_r(3)];

%                 fi = obj.Ud./[obj.ganhoft]; % fi desejado 
%                 Ksir_d = obj.Modcin*(fi); % converte para comandos de velocidade (apenas para registro e plots futuros)
                
                obj.plotInfo.Pvel(:,iteracao+1) = Ksir_d; % atualiza o vetor das velocidades (comandos) do rob? durante o experimento.
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
%                 obj.plotInfo.Pu(:,iteracao+1) = obj.Ud; % U = [U_d ; U_e] de -1 a 1 % valor desejado
                obj.plotInfo.Pu(:,iteracao+1) = Ud; % U = [U_d ; U_e] de -1 a 1 % valor desejado         
                obj.plotInfo.Pu_real(:,iteracao+1) = Ureal; % U = [U_d ; U_e] de -1 a 1 % valor real
                obj.plotInfo.Pfi(:,iteracao+1) = fi; % U = [Fi_d ; Fi_e] % valor desejado de fi
                obj.plotInfo.Pfi_real(:,iteracao+1) = FIreal; % U = [Fi_d ; Fi_e] % valor real de fi
                
            else
                
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = 0;
                Wmedido = 0;

                FIreal = obj.Modcin\[Vmedido ; Wmedido];
                Ureal = [obj.ganhoft].*FIreal;

                fi = obj.Ud./[obj.ganhoft]; % fi desejado 
                Ksir_d = obj.Modcin*(fi); % converte para comandos de velocidade (apenas para registro e plots futuros)

                
                obj.plotInfo.Pvel(:,iteracao+1) = Ksir_d; % atualiza o vetor das velocidades (comandos) do rob? durante o experimento.
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
                obj.plotInfo.Pu(:,iteracao+1) = obj.Ud; % U = [U_d ; U_e] de -1 a 1 % valor desejado
                obj.plotInfo.Pu_real(:,iteracao+1) = Ureal; % U = [U_d ; U_e] de -1 a 1 % valor real
                obj.plotInfo.Pfi(:,iteracao+1) = fi; % U = [Fi_d ; Fi_e] % valor desejado de fi
                obj.plotInfo.Pfi_real(:,iteracao+1) = FIreal; % U = [Fi_d ; Fi_e] % valor real de fi
            end
                
        end
        
        function obj = simulacao_apenas_cinematica(obj,tamos,iteracao)
            
            if ~obj.falhou
                R = [cos(obj.Pos(3)) sin(obj.Pos(3)) 0 ; -sin(obj.Pos(3)) cos(obj.Pos(3)) 0 ; 0 0 1]; % matriz de rota??o
                Ksi_I = R\obj.Ksi_r; % coloca as velocidades V e W reais no SC do ambiente
                obj.Pos = obj.Pos + Ksi_I*tamos; % atualiza??o da posi??o do rob? (integra??o no SC do ambiente)
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = obj.Ksi_r(1);
                Wmedido = obj.Ksi_r(3);
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
            else
                obj.hPos(:,1:end-1) = obj.hPos(:,2:end);
                obj.hPos(:,end) = obj.Pos;
                % converte theta para -pi a pi
                if obj.Pos(3) > pi, obj.Pos(3) = obj.Pos(3) - 2*pi; end
                if obj.Pos(3) < -pi, obj.Pos(3) = obj.Pos(3) + 2*pi; end
                Vmedido = 0;
                Wmedido = 0;
                obj.plotInfo.Pvel_medido(:,iteracao+1) = [Vmedido ; Wmedido]; % atualiza o vetor das velocidades reais do rob? durante o experimento.
                obj.plotInfo.P(:,iteracao+1) = obj.Pos; % atualiza o vetor das posi??es do rob? durante o experimento (SC do ambiente).
            end
        end
        
        function obj = simulacao_sensores(obj,A)
           
            %%% tratamento dos sensores (antiga função inicio)
            Ps_i = obj.s_i;
            [ymaxA , xmaxA] = size(A);
            smax = obj.saturacao*(ones(1,length(obj.angs)));
            smax = smax + obj.raio;
            %s é um vetor cujos elementos representam o ponto da leitura máxima (a priori) para
            %cada um dos sensores no sistema de coordenadas do robô
            obj.s = [smax.*cos(obj.angs) ; smax.*sin(obj.angs)]; % vetor no SC do robô (ainda saturado)    
            Ri = [cos(obj.Pos(3)) -sin(obj.Pos(3)); sin(obj.Pos(3)) cos(obj.Pos(3))]; % matriz de rotação
            %s_i é o vetor s colocado no sistema de coordenadas do ambiente (sem ruído) (ainda saturado)
            obj.s_i = Ri*obj.s;
            obj.s_i(1,:) = obj.s_i(1,:) + obj.Pos(1);
            obj.s_i(2,:) = obj.s_i(2,:) + obj.Pos(2);

            pos_sensor = [obj.raio.*cos(obj.angs) ; obj.raio.*sin(obj.angs)];
            pos_sensor = Ri*pos_sensor;
            pos_sensor(1,:) = pos_sensor(1,:) + obj.Pos(1);
            pos_sensor(2,:) = pos_sensor(2,:) + obj.Pos(2);

            obj.sensorRobos = [];
            obj.sensorObstaculo = [];
            for k=1:length(obj.angs)
                Vsx = round(linspace(pos_sensor(1,k),obj.s_i(1,k),obj.saturacao));
                Vsy = round(linspace(pos_sensor(2,k),obj.s_i(2,k),obj.saturacao));
                Vs = [Vsx ; Vsy];
                for j=1:obj.saturacao
                    if (Vs(1,j)>0 && Vs(1,j)<=xmaxA && Vs(2,j)>0 && Vs(2,j)<=ymaxA) % se a leitura esta dentro do mapa
                        if A(Vs(2,j),Vs(1,j)) ~= 255
                            obj.s_i(1,k) = Vs(1,j);
                            obj.s_i(2,k) = Vs(2,j);
                            if A(Vs(2,j),Vs(1,j)) == 50
                                obj.sensorRobos = [obj.sensorRobos k];
                            end
                            if A(Vs(2,j),Vs(1,j)) == 0
                                obj.sensorObstaculo = [obj.sensorObstaculo k];
                            end
                            break;
                        end
                    end
                end
            end
            %retorna a medição para o sistema de coordenadas do robô
            % O ROBÔ É ORIENTADO PARA O EIXO X.
            Ps_i(1,:) = obj.s_i(1,:) - obj.Pos(1);
            Ps_i(2,:) = obj.s_i(2,:) - obj.Pos(2);
            obj.s = Ri\Ps_i;
            %adiciona ruído na medição (a resolução da medição depende da resolução do
            %mapa)
            obj.s = obj.s + obj.ruido*randn(2,length(obj.angs));
            % sensor com ruido de medição no S.C. do ambiente
            aux = Ri*obj.s;
            obj.s2(1,:) = aux(1,:) + obj.Pos(1);
            obj.s2(2,:) = aux(2,:) + obj.Pos(2);

            % tratamento dos sensores (antiga função FIM)

            obj.v_sensor = sqrt( (obj.s2(1,:)-pos_sensor(1,:)).^2 + (obj.s2(2,:)-pos_sensor(2,:)).^2 ); 
            v_colidiu = sqrt( (obj.s_i(1,:)-pos_sensor(1,:)).^2 + (obj.s_i(2,:)-pos_sensor(2,:)).^2 ); % distância real entre o robô e os obstaculos
            ds0 = sort(v_colidiu);  % ds0 é a distância do sensor sem ruído com a menor leitura
            if ds0(1) <= 1 %colisão de pior caso com 
                obj.colidiu = 1;
            end
            
        end
        
        function obj = configuracao_inicial(obj,experimento,tempo_max,k)     
            
            obj.colidiu = 0; % verifica se o rob? colidiu para finalizar a simula??o
            obj.falhou = 0;
            tamos = experimento.tamos; % tempo de amostragem da simula??o em segundos
            % inicializa??o da posi??o do rob?
            x = experimento.rbx(k);
            y = experimento.rby(k);
            theta = experimento.ang(k)*pi/180; % inicia orientado para o eixo x do plano
            obj.Pos = [x ; y ; theta]; % ? a posi??o atual do rob? (aqui ? a posi??o inicial).
            obj.hPos = ones(3,500).*obj.Pos;
            % Inicializa Posição de destino
            obj.Pdes = [experimento.dx(k) ; experimento.dy(k)];
            
            % vetores sobre o experimento
            obj.plotInfo.P = zeros(3,round(tempo_max/tamos)-1);
            obj.plotInfo.P(:,1) = obj.Pos;  
            obj.plotInfo.Pvel = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pvel_medido = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pu = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pu_real = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pfi = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.Pfi_real = zeros(2,round(tempo_max/tamos)-1);
            obj.plotInfo.tempos  = [];
            obj.X = [0 ; 0]; % estado do sistema para simula??o din?mica

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % ?NGULOS NO SISTEMA DE COORDENADAS DO ROB?! -> 0 graus ? o eixo X (eixo de movimenta??o)
            resang = 5;
            obj.angs = [0:resang:360-resang]*pi/180;

            % inicializa??o dos sensores com zero;
            obj.s = zeros(2,length(obj.angs));
            obj.s_i = obj.s;
            obj.s2 = obj.s; %sensor com ru?do de medi??o no SC do ambiente          
        end
        
        function obj = plotConfig(obj,k)
            
            % [vlimitex,vlimitey] = scircle2(0,0,1,0); % circulo limite de vis?o MATLAB
            aux = linspace(0,2*pi*100,100);
            vlimitex = cos(aux)';
            vlimitey = sin(aux)';
             % plot do robô em terceira pessoa
            xc = vlimitex.*obj.raio;
            yc = vlimitey.*obj.raio;

            pxyc = [cos(obj.Pos(3)) -sin(obj.Pos(3)) ; sin(obj.Pos(3)) cos(obj.Pos(3))]*[xc' ; yc'];
            xc3 = pxyc(1,:)+obj.Pos(1);
            yc3 = pxyc(2,:)+obj.Pos(2);
            obj.plt.a = plot(xc3,yc3,'b');
            obj.plt.b = plot([obj.Pos(1) obj.Pos(1)+obj.raio*cos(obj.Pos(3))],[obj.Pos(2) obj.Pos(2)+obj.raio*sin(obj.Pos(3))],'r');
            obj.plt.c = plot(obj.s2(1,:),obj.s2(2,:),'.','MarkerEdgeColor',obj.colors(5),'MarkerSize',7);
            obj.plt.e = plot(obj.hPos(1,:),obj.hPos(2,:),obj.colors(k));

        end
        
        function obj = vecCorrecao(obj,iteracao, habilitaDinamica)
            if habilitaDinamica==1
                obj.plotInfo.P = obj.plotInfo.P(:,1:iteracao);
                obj.plotInfo.Pvel = obj.plotInfo.Pvel(:,1:iteracao);
                obj.plotInfo.Pvel_medido = obj.plotInfo.Pvel_medido(:,1:iteracao);
                obj.plotInfo.Pu = obj.plotInfo.Pu(:,1:iteracao);
                obj.plotInfo.Pu_real = obj.plotInfo.Pu_real(:,1:iteracao);
                obj.plotInfo.Pfi = obj.plotInfo.Pfi(:,1:iteracao);
                obj.plotInfo.Pfi_real = obj.plotInfo.Pfi_real(:,1:iteracao);
            else
                obj.plotInfo.P = obj.plotInfo.P(:,1:iteracao);
                obj.plotInfo.Pvel_medido = obj.plotInfo.Pvel_medido(:,1:iteracao);
            end
        end 
        
        function obj = simulacao_falha(obj)
                obj.falhou = 1;
        end
        
        
%         function dentro = pontoDentroPoligono(obj, ponto, vertices)
%             x = ponto(1);
%             y = ponto(2);
%             n = size(vertices, 1);
%             dentro = false;
% 
%             for i = 1:n
%                 xi = vertices(i, 1);
%                 yi = vertices(i, 2);
%                 xj = vertices(mod(i, n) + 1, 1);
%                 yj = vertices(mod(i, n) + 1, 2);
% 
%                 % Verifica se o ponto está entre os valores y das arestas
%                 if (yi > y) ~= (yj > y)
%                     % Calcula a interseção do raio horizontal com a aresta
%                     intersec_x = (xj - xi) * (y - yi) / (yj - yi) + xi;
%                     if x < intersec_x
%                         dentro = ~dentro;
%                     end
%                 end
%             end
%         end
        function dentro = pontosDentroPoligono(obj,pontos, vertices)
            % Obter o tamanho da matriz de pontos
            [numRows, numCols] = size(pontos(:,:,1));
            % Inicializar a matriz lógica de saída
            dentro = false(numRows, numCols);

            % Obter as coordenadas x e y dos pontos
            x = pontos(:,:,1);
            y = pontos(:,:,2);

            % Obter o número de vértices do polígono
            n = size(vertices, 1);

            for i = 1:n
                xi = vertices(i, 1);
                yi = vertices(i, 2);
                xj = vertices(mod(i, n) + 1, 1);
                yj = vertices(mod(i, n) + 1, 2);

                % Verifica se os pontos estão entre os valores y das arestas
                cond = (yi > y) ~= (yj > y);

                % Calcula a interseção do raio horizontal com a aresta
                intersec_x = (xj - xi) * (y - yi) ./ (yj - yi) + xi;

                % Atualiza a matriz lógica para os pontos onde a condição é verdadeira
                dentro = dentro ~= (cond & (x < intersec_x));
            end
        end
        
        
        
        
        function Mapa_depois = DescobrimentoMapa(obj,Mapa_antes)
            Mapa_depois = Mapa_antes;
            % Obter o tamanho da imagem
            [rows, cols] = size(Mapa_antes);
            
            

            % Criar uma grade de coordenadas
%             [x, y] = meshgrid(1:cols, 1:rows);
            xx = round(obj.Pos(2) - (obj.raio + obj.saturacao));
            xx2 = round(obj.Pos(2) + (obj.raio + obj.saturacao));
            yy = round(obj.Pos(1) - (obj.raio + obj.saturacao));
            yy2 = round(obj.Pos(1) + (obj.raio + obj.saturacao));
            if(xx <= 0) xx = 1; end
            if(yy <= 0) yy = 1; end
            if(xx2 > rows) xx2 = rows; end
            if(yy2 > cols) yy2 = cols; end
            [x, y] = meshgrid(yy:yy2, xx:xx2);
%             x = x(xx:xx2,yy:yy2);
%             y = y(xx:xx2,yy:yy2);
            % Marcação de obstaculos
            for k = obj.sensorObstaculo
                 Mapa_depois(round(obj.s2(2,k)),round(obj.s2(1,k))) = 0;
            end
%             Marcação de região vizitada
%             Calcular a distância de cada ponto ao centro
%             distance = sqrt((x - obj.Pos(1)).^2 + (y - obj.Pos(2)).^2);
%             Criar uma máscara para a circunferência preenchida
%             [d_obs_min1, idc_min1] = min(obj.v_sensor);
%             mask = (distance <= (d_obs_min1));
%             for c = 1:cols
%                  for l = 1:rows 
%                    mask(l,c) = obj.pontoDentroPoligono([c,l],(obj.s2)');
%                  end
%             end
             pontos(:,:,1)=x;
             pontos(:,:,2)=y;
             mask = false(rows, cols);
             mask(xx:xx2,yy:yy2) = obj.pontosDentroPoligono(pontos, (obj.s2)');
            [By,Bx]=find(mask == 1);
            for b = 1:length(Bx)
                 if (Mapa_depois(By(b),Bx(b))~=0 )
                     Mapa_depois(By(b),Bx(b)) = 255;
                 end
            end
           
        end
        
        
    end
end