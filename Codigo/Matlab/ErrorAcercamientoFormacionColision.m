% =========================================================================
%
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 25/08/2022
% Basado en:"Simulación de control de formación"
% de Andrea Maybell Peña Echeverría
% =========================================================================
%
% =========================================================================

cantI = 100;                    % cantidad de simulaciones a realizar
EIndividual = zeros(cantI,8);  % energía individual por agente en cada simulación
ETotal = zeros(1,cantI);        % energía total en cada simulación
EI = zeros(1,cantI);            % error individual en cada simulación
ExitoTotalF = 0;                % cantidad de formaciones 100% exitosas
Exito9F = 0;                    % cantidad de formaciones 90% exitosas
Exito8F = 0;                    % cantidad de formaciones 80% exitosas
Exito7F = 0;                    % cantidad de formaciones 70% exitosas
Fail = 0;                       % cantidad de formaciones fallidas

for I = 1:cantI
    %% Inicialización del mundo
    gridsize = 10;      % tamaño del mundo
    initsize = 10;
    N = 8;             % número de agentes
    dt = 0.01;          % período de muestreo
    T = 20;             % tiempo final de simulación

 
    % Inicialización de la posición de los agentes
    X = initsize*rand(3,N);
    Xi = X;

    % Inicialización de la velocidad de los agentes
    V = zeros(3,N);

    %% Selección matriz y parámetros del sistema
    Form = 2;
    d = MatrizF(Form);    % matriz de formación
    r = 1;               % radio agentes
    R = 10;              % rango sensor
%     VelMax = 10;         % velocidad máxima
    %% Inicialización simulación
    t = 0;                      % inicialización de tiempo
    ciclos = 1;                 % cuenta de la cantidad de ciclos 
    historico = zeros(100*T,N); % histórico de velocidades
    cambio = 0;                 % variable para el cambio de control
    
    while(t < T)
       for i = 1:N
            E = 0;
            for j = 1:N
                dist = X(:,i)- X(:,j); % vector xi - xj
                mdist = norm(dist);    % norma euclidiana vector xi - xj
                dij = 2*d(i,j);        % distancia deseada entre agentes i y j

                % Peso añadido a la ecuación de consenso
                if(mdist == 0)
                    w = 0;
                else
                    switch cambio
                        case 0
                            w = (mdist - (2*(r + 1.5)))/(mdist - (r + 1.5))^2;
                        case 1
                            w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)^2)/(mdist*(mdist - r)^2); 
                    end 
                end
            end 
            % Tensión de aristas entre agentes
            E = E + w.*dist;
        end

        % Al llegar muy cerca de la posición deseada realizar cambio de control
%         if(norm(V) < 0.2)
%             cambio = cambio + 1;
%         end
        % Actualización de la posición de los agentes
        X = X + V*dt;
        
        % Almacenamiento de variables
        historico(ciclos,:) = (sum(V.^2,1)).^0.5;
        
        % Actualización de tiempo
        t = t + dt;
        ciclos = ciclos + 1;
    end
    
    %% Cálculo del error final
    mDistF = 0.5*DistEntreAgentes(X);
    errorF = ErrorForm(mDistF,MatrizF(Form)); % error de formación simulación I
    energiaI = sum(historico.*dt,1);            % energía individual simulación I
    energiaT = sum(energiaI,2);                 % energía total simulación I
    
    %% Porcentaje éxito formación
    % Una formación se considera exitosa con un error cuadrático medio 
    % menor a 0.05
    if(errorF > 0.05)   
        % Si la formación no fue exitosa se evalua el éxito individual de 
        % de cada agente. Un agente llegó a la posición deseada si tiene un
        % porcentaje de error menor al 15%.
        [errorR,cantAS] = ErrorIndividual(mDistF, MatrizF(Form), 15);
    else
        % El que la formación haya sido exitosa implica que todos los
        % agentes llegaron a la posición deseada
        errorR = errorF;    % error de formación relativo
        cantAS = N;         % cantidad de agentes que llegan a la posición deseada
    end
    
    %% Porcentaje de agentes en posición deseada
    % Si el error de formación sin tomar en cuenta a los agentes que se
    % alejaron considerablemente, es menor a 0.05 implica que hubo un
    % porcentaje de la formación que sí se logró. 
    if(errorR < 0.05)
        if(cantAS == N)                     % formación 100% exitosa
            ExitoTotalF = ExitoTotalF + 1;
        elseif (cantAS == round(N*0.9))     % formación 90% exitosa
            Exito9F = Exito9F + 1;
        elseif (cantAS == round(N*0.8))     % formación 80% exitosa     
            Exito8F = Exito8F + 1;
        elseif (cantAS == round(N*0.7))     % formacion 70% exitosa
            Exito7F = Exito7F + 1;
        else                                % formación fallida
            Fail = Fail +1;
        end
    else
        Fail = Fail + 1;
    end
        
    VResults = [ExitoTotalF, Exito9F, Exito8F, Exito7F, Fail];
end

% Guardar resultados como resulte más conveniente
