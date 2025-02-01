clc
clear 
close all


nRobos = 3; % N�mero de rob�s


%% Posi��es de destino desejada 
experimento.dx = 200 + 100*cos(pi*([1:nRobos] - 1)/(nRobos-1)); %Posi��o de destino no eixo x [cm]
experimento.dy = 100 + 100*sin(pi*([1:nRobos] - 1)/(nRobos-1)); %Posi��o de destino no eixo y [cm]


%% Configura��es das caracter�sticas do experimento simulado (Apenas para SIMULA��O).
experimento.rbx(1:nRobos) = 200 + 95*cos(pi*([1:nRobos] - 1)/(nRobos-1));
experimento.rby(1:nRobos) = 100 + 95*sin(pi*([1:nRobos] - 1)/(nRobos-1));
experimento.ang(1:nRobos) = zeros(1,nRobos);  %orienta��o do rob� em rela��o ao eixo x do ambiente [graus] (apenas para a simula��o)
experimento.tamos = 0.01; %tempo de amostragem em segundos (10 ms) (apenas para a simula��o)
mapabmp = 'mapa_artigo.bmp'; % mapa do ambiente simulado (apenas para a simula��o)

%% Posi��es de destino desejada 
experimento.dx = experimento.rbx; %Posi��o de destino no eixo x [cm]
experimento.dy = experimento.rby; %Posi��o de destino no eixo y [cm]
% experimento.dx(3) = 300;
% experimento.dy(3) = 420;
% experimento.dx(2) = 800;
% experimento.dy(2) = 250;
% experimento.dx(1) = 730;
% experimento.dy(1) = 40;
%    checks = []    ;

%% Carregando o modelo Kinodin�mico do Zuadento 
% configuracao_robo
for i = 1:nRobos
    robo_(i) = robo(i);
end
experimento.mapa_descoberto = [];

%% Par�metros da simula��o
tempo_max = 1000; % tempo m�ximo do experimento em segundos 
tempo_total = tic; % controle de custo computacional (debug apenas)
%% Habilita Plot
habilitaPlot = 1;

%% Habilita Din�mica
habilitaDinamica = 0;
%% Simulador do Zuadento
ZUADENTO_SIMULADOR(experimento,robo_,mapabmp,tempo_max,habilitaPlot,habilitaDinamica,nRobos);

toc(tempo_total) % controle de custo computacional (debug apenas)
% Fun��es para vizualiza��o do resultado
% load('experimento_aleat_distPond_5.mat')
% funcao_plotar_caminho_robo('experimento_aleat_distPond_5.mat',nRobos)
% funcao_plotar_graficos('experimento_aleat_distPond_5.mat',nRobos)
% funcao_resultado('experimento_aleat_distPond_5.mat', nRobos)
% figure()
% imshow(uint8(experimento.mapa_descoberto(end:-1:1,:)))

