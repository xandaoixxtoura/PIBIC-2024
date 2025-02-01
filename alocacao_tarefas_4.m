if(experimento.flag)
    
   se = ones(5,5);
   [experimento.fronteiray,experimento.fronteirax] = find(edge(experimento.mapa_descoberto, 'Sobel')&(~imdilate(experimento.mapa_descoberto==0,se)));

   [pontos] = kmeans_([experimento.fronteirax,experimento.fronteiray], nRobos,10);
                    
    Mdist = [];
    for k = 1:nRobos
        Mdist = [Mdist; sqrt((robo_(k).Pos(1)- pontos(1,:)).^2 + (robo_(k).Pdes(2)- pontos(2,:)).^2)];
    end
    Mdist = Mdist';
    list = 1:nRobos;
    A_ = perms(list); 
    L = [];
    for l = 1:size(A_,1)
        L = [L sum(diag(Mdist(A_(l,:),:)))];
    end
    [~,idc]= min(L);
    A_ = A_(idc,:);
    for k = 1:nRobos
        robo_(k).Pdes = pontos(:,A_(k));
        robo_(k).chegou = false;
    end
end