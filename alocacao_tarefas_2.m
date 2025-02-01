if(experimento.flag)
    
   se = ones(5,5);
   [experimento.fronteiray,experimento.fronteirax] = find(edge(experimento.mapa_descoberto, 'Sobel')&(~imdilate(experimento.mapa_descoberto==0,se)));
   
 
   [obsy,obsx] = find(experimento.mapa_descoberto==0);
   k = 1;
   while k<(nRobos+1)
        idx = randi([1 length(experimento.fronteiray)],1,1);
        dist_obs = sqrt((experimento.fronteirax(idx)-obsx).^2 + (experimento.fronteiray(idx)-obsy).^2);
        if min(dist_obs)>robo_(k).raio
            if k == 1
                pontos(:,k) = [experimento.fronteirax(idx);experimento.fronteiray(idx)];
                k = k+1;
            else
                dist_p = sqrt((experimento.fronteirax(idx)-pontos(1,1:(k-1))).^2 + (experimento.fronteiray(idx)-pontos(2,1:(k-1))).^2);
                if min(dist_p)>robo_(k).raio
                    pontos(:,k) = [experimento.fronteirax(idx);experimento.fronteiray(idx)];
                    k = k+1;                    
                end
            end
        end
    end
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