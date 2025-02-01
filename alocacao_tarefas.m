if(experimento.flag)
    
     se = ones(5,5);
   [fronteiray,fronteirax] = find(edge(experimento.mapa_descoberto, 'Sobel')&(~imdilate(experimento.mapa_descoberto==0,se)));

%    [pontos] = kmeans_([fronteirax, fronteiray], nRobos,10);
%    end
   [obsy,obsx] = find(experimento.mapa_descoberto==0);
   k = 1;
   while k<(nRobos+1)
        idx = randi([1 length(fronteiray)],1,1);
        dist_obs = sqrt((fronteirax(idx)-obsx).^2 + (fronteiray(idx)-obsy).^2);
        if min(dist_obs)>robo_(k).raio
            if k == 1
                pontos(:,k) = [fronteirax(idx);fronteiray(idx)];
                k = k+1;
            else
                dist_p = sqrt((fronteirax(idx)-pontos(1,1:(k-1))).^2 + (fronteiray(idx)-pontos(2,1:(k-1))).^2);
                if min(dist_p)>robo_(k).raio
                    pontos(:,k) = [fronteirax(idx);fronteiray(idx)];
                    k = k+1;                    
                end
            end
        end
    end
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