clc
clear 
close all


nRobos = 3; % Número de robôs


%% Posições de destino desejada 
experimento.dx = 200 + 100*cos(pi*([1:nRobos] - 1)/(nRobos-1)); %Posição de destino no eixo x [cm]
experimento.dy = 100 + 100*sin(pi*([1:nRobos] - 1)/(nRobos-1)); %Posição de destino no eixo y [cm]


%% Configurações das características do experimento simulado (Apenas para SIMULAÇÃO).
experimento.rbx(1:nRobos) = 200 + 95*cos(pi*([1:nRobos] - 1)/(nRobos-1));
experimento.rby(1:nRobos) = 100 + 95*sin(pi*([1:nRobos] - 1)/(nRobos-1));
experimento.ang(1:nRobos) = zeros(1,nRobos);  %orientação do robô em relação ao eixo x do ambiente [graus] (apenas para a simulação)
experimento.tamos = 0.01; %tempo de amostragem em segundos (10 ms) (apenas para a simulação)
mapabmp = 'mapa_artigo.bmp'; % mapa do ambiente simulado (apenas para a simulação)

%% Posições de destino desejada 
experimento.dx = experimento.rbx; %Posição de destino no eixo x [cm]
experimento.dy = experimento.rby; %Posição de destino no eixo y [cm]
% experimento.dx(3) = 300;
% experimento.dy(3) = 420;
% experimento.dx(2) = 800;
% experimento.dy(2) = 250;
% experimento.dx(1) = 730;
% experimento.dy(1) = 40;
%    checks = []    ;

%% Carregando o modelo Kinodinâmico do Zuadento 
% configuracao_robo
for i = 1:nRobos
    robo_(i) = robo(i);
end
experimento.mapa_descoberto = [];

%% Parâmetros da simulação
tempo_max = 1000; % tempo máximo do experimento em segundos 
tempo_total = tic; % controle de custo computacional (debug apenas)
%% Habilita Plot
habilitaPlot = 1;

%% Habilita Dinâmica
habilitaDinamica = 0;
%% Simulador do Zuadento
ZUADENTO_SIMULADOR(experimento,robo_,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos);

toc(tempo_total) % controle de custo computacional (debug apenas)
% Funções para vizualização do resultado
% load('experimento_aleat_distPond_5.mat')
% funcao_plotar_caminho_robo('experimento_aleat_distPond_5.mat',nRobos)
% funcao_plotar_graficos('experimento_aleat_distPond_5.mat',nRobos)
% funcao_resultado('experimento_aleat_distPond_5.mat', nRobos)
% figure()
% imshow(uint8(experimento.mapa_descoberto(end:-1:1,:)))

