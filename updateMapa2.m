function A = updateMapa2(Mapa,robo,nRobos,idx)
    %updateMapa Summary of this function goes here
    Bposx = []; % Posição x do corpo do robô
    Bposy = []; % Posição y do corpo do robô
    vec = 1:nRobos;
    for k = setdiff(vec,idx)
        Bposx = [Bposx; round(robo(k).Pos(1) + robo(k).raio*cos(2*pi*[0:99]/100))];
        Bposy = [Bposy; round(robo(k).Pos(2) + robo(k).raio*sin(2*pi*[0:99]/100))];
    end
    N = length(Bposx);
    A = Mapa;
    for k = 1:nRobos
       I = find(setdiff(vec,idx)==k);
       N = length(Bposx(I,:));
       for l = 1:N
           if ((~isnan(Bposx(I,l)))||(~isnan(Bposy(I,l))))
            A(Bposy(I,l),Bposx(I,l)) = 50;
           end
       end
    end
end
