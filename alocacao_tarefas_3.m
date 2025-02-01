if(experimento.flag)
    
   se = ones(5,5);
   [fronteiray,fronteirax] = find(edge(experimento.mapa_descoberto, 'Sobel')&(~imdilate(experimento.mapa_descoberto==0,se)));

   [pontos] = kmeans_([fronteirax, fronteiray], nRobos,10);
                    
    Mdist = [];
    for k = 1:nRobos
        Mdist = [Mdist; sqrt((robo_(k).Pos(1)- pontos(1,:)).^2 + (robo_(k).Pdes(2)- pontos(2,:)).^2)];
    end
    
    D = 3;
    while D > 0
        [v,I]=min(Mdist);
        [~,J]=min(v);
         Pdes(I(J),:) = [pontos(1,J), pontos(2,J)];
         Mdist(I(J),:) = 1e10;
         Mdist(:,J) = 1e10;
         D = D - 1;
    end
    for k = 1:nRobos
        robo_(k).Pdes = Pdes(k,:)';
        robo_(k).chegou = false;
    end
end