% =========================================================================
% SIMULACIÓN DEL PROBLEMA DE FORMACIÓN EN 3D CON CONTROL DE COLISIONES
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 10/16/2022
% Basado en:"Simulación de control de formación"
% de Andrea Maybell Peña Echeverría
% =========================================================================
% El siguiente script implementa la simulación de la ecuación modificada
% de consenso para el caso del consenso en una formación con evasión de
% colisiones. Solo proporciona datos estadísticos e información numérica,
% no incluye la simulación visual.
% =========================================================================

cantI = 100;                    % cantidad de iteraciones a realizar
EIndividual = zeros(cantI,8);   % energía del agente en cada simulación
ETotal = zeros(1,cantI);        % energía total de los agentes por simulación
EI = zeros(1,cantI);            % error individual en cada simulación
ExitoTotalF = 0;                % cantidad de formaciones 100% exitosas
Fail = 0;                       % cantidad de formaciones fallidas
Energia50 = 0;
Energia100 = 0;
Energia200 = 0;
Energia300 = 0;
Enerprom = 0;
Promedio = 0;
for I = 1:cantI
    %% Inicialización del mundo
    gridsize = 10;      % tamaño del espacio tridimensional
    initsize = 10;
    N = 8;          % Definir la cantidad de agentes
    dt = 0.01;      % Tiempo de muestreo
    T = 20;         % Tiempo máximo de simulación
    
    % Inicialización de la posición de los agentes
    X = initsize*rand(3,N);
    Xi = X;
    
    % Inicialización de la velocidad de los agentes
    V = zeros(3,N);
    
    %% Selección matriz y parámetros del sistema
    Formacion = 2;
    d = MatrizF(Formacion); % Selección de formación
    r = 1;                  % Radio físico de los agentes
    
    %% Inicialización simulación
    t = 0;                      
    ciclos = 1;                 % Contador de ciclos para finalizar la formación 
    historico = zeros(100*T,N); % Variable que almacena la velocidad
    cambio = 0;                 % Variable para el cambio de ecuación de consenso
       
    while(t < T)
       for i = 1:N
            E = 0;
            for j = 1:N
            dist = X(:,i)- X(:,j); % vector de distancia entre agente i y j
            mdist = norm(dist);    % norma euclidiana vector xi - xj
            dij = 2*d(i,j);        % distancia deseada entre agentes i y j
            
            % Se calcula el peso para la ecuación
            if(mdist == 0)
                w = 0;
            else
                switch cambio
                    case 0
                        % Ecuación de consenso modificada para acercamiento
                        w = (mdist - (2*(r + 1)))/(mdist - (r + 1))^2;
                    case 1
                        % Ecuación de consenso modificada para evitar
                        % colisiones y formarse
                        w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)^2)/(mdist*(mdist - r)^2); 
                end 
            end
            % Tensión de aristas entre agentes
            E = E + w.*dist;
        end
        
        % Actualización de velocidad
        V(:,i) = -1*E;
        
        end

        % Al llegar muy cerca de la posición deseada realizar cambio de control
        if(norm(V) < 2)
            cambio = 1;
        end
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
    errorF = ErrorForm(mDistF,MatrizF(Formacion)); % error de formación simulación I
    energiaI = sum(historico.*dt,1);            % energía individual simulación I
    energiaT = sum(energiaI,2);                 % energía total simulación I
    Enerprom = Enerprom + energiaT;
    Promedio = Enerprom / 100;
    if energiaT < 50
        Energia50 = Energia50 + 1;
    end
    if energiaT < 100 && energiaT> 50
        Energia100 = Energia100 + 1;
    end
    if energiaT < 200 && energiaT>100
        Energia200 = Energia200 + 1;
    end
    if energiaT>200
        Energia300 = Energia300 + 1;
    end
    
    %% Porcentaje éxito formación
    % Una formación se considera exitosa con un error cuadrático medio 
    % menor a 0.05
    if(errorF > 0.05)   
        % Si la formación no fue exitosa se evalua el éxito individual de 
        % de cada agente. Un agente llegó a la posición deseada si tiene un
        % porcentaje de error menor al 15%.
        [errorR,cantAS] = ErrorIndividual(mDistF, MatrizF(Formacion), 15);
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
        else                                % formación fallida
            Fail = Fail +1;
        end
    else
        Fail = Fail + 1;
    end
        
    VResults = [ExitoTotalF, Fail];
end

% Guardar resultados como resulte más conveniente
