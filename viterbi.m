function [Vero, estadoEstimado] = viterbi(Y,P0,A,B,O)
    % Obeserva��es: y
    % Par�metros do HMM: P0, A, B
    % Conjunto de observa��es/emiss�es poss�veis: O
    % Numero de estados poss�veis: K

    T = length(Y); % Numero de observa��es (contando com inicio e fim)
    [K, N] = size(B);
    estadosAnteriores = zeros(K,T);
    maxVero = zeros(K,T);

    t = 1;
    posObs = find(Y(t)==O);
    maxVero(:,t) = log(P0)+log(B(:,posObs));
    for t = 2:T
       posObs = find(Y(t)==O);
       for estado = 1:K
          [val,pos]=max(log(A(:,estado))+maxVero(:,t-1));
          maxVero(estado,t) = val + log(B(estado,posObs));
          estadosAnteriores(estado,t) = pos;
       end
    end
    
    estadoEstimado = zeros(1,T);
    [Vero,estadoEstimado(T)] = max(maxVero(:,T));
    
    for t=T:-1:2
        estadoEstimado(t-1) = estadosAnteriores(estadoEstimado(t),t); 
    end
    
end