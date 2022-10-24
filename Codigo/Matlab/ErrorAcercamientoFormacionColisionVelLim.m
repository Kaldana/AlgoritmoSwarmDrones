% =========================================================================
% SIMULACIÓN DEL PROBLEMA DE FORMACIÓN EN 3D CON CONTROL DE COLISIONES Y
% VELOCIDAD
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 10/16/2022
% Basado en:"Simulación de control de formación"
% de Andrea Maybell Peña Echeverría
% =========================================================================
% El siguiente script implementa la simulación de la ecuación modificada
% de consenso para el caso del consenso en una formación con evasión de
% colisiones y límite de velocidad. Solo proporciona datos estadísticos e
% información numérica, no incluye la simulación visual.
% =========================================================================

cantI = 100;                    % cantidad de iteraciones a realizar
EIndividual = zeros(cantI,8);   % energía del agente en cada simulación
ETotal = zeros(1,cantI);        % energía total de los agentes por simulación
EI = zeros(1,cantI);            % error individual en cada simulación
ExitoTotalF = 0;                % cantidad de formaciones 100% exitosas
Exito9F = 0;                    % cantidad de formaciones 90% exitosas
Exito8F = 0;                    % cantidad de formaciones 80% exitosas
Exito7F = 0;                    % cantidad de formaciones 70% exitosas
Fail = 0;                       % cantidad de formaciones fallidas

for I = 1:cantI
    %% Inicialización del mundo
    gridsize = 10;  % tamaño del espacio tridimensional
    initsize = 10;
    N = 8;          % Definir la cantidad de agentes
    dt = 0.01;      % Tiempo de muestreo
    T = 20;         % Tiempo máximo de simulación
 
    % Inicialización de la posición de los agentes
    X = initsize*rand(3,N);
    
    % El siguiente ciclo sirve para colocar a todos los agentes en el plano
    % XY como se realizaría en pruebas físicas, con la idea de realizar la
    % simulación lo más real posible.
    
    for s = 1:N
    X(3,s) = 0  ;
    end

    % AGENTES EN PLANO ARBITRARIO
    
    % X(1,1) = 0; X(1,2) = -4; X(1,3) = 0; X(1,4) = -4; 
    % X(1,5) = 0; X(1,6) = -4; X(1,7) = 0; X(1,8) = 15;
    % 
    % X(2,1) = -4;X(2,2) = -4; X(2,3) = -2;X(2,4) = -2;
    % X(2,5) = 0; X(2,6) = 0;  X(2,7) = 2; X(2,8) = 15;
    % 
    % X(3,1) = 2; X(3,2) = 2; X(3,3) = 5; X(3,4) = 5;
    % X(3,5) = 8; X(3,6) = 8; X(3,7) = 11; X(3,8) = 15;

    Xi = X; % Vector de posición de los agentes

    % Inicialización de la velocidad de los agentes
    V = zeros(3,N);

    %% Selección matriz y parámetros del sistema
    Formacion = 2;
    d = MatrizF(Formacion); % Selección de formación
    r = 1;                  % Radio físico de los agentes
    VelMax = 10;            % Límite de velocidad
    
    %% Inicialización simulación
    t = 0;                      
    ciclos = 1;                 % Contador de ciclos para finalizar la formación 
    historico = zeros(100*T,N); % Variable que almacena la velocidad
    cambio = 0;                 % Variable para el cambio de ecuación de consenso

    %% Dinámica inicial de los agentes
    % Este bloque permite que los drones se eleven durante 3 unidades de
    % tiempo hacia arriba para que puedan salir del plano y así partir de
    % una singularidad. NOTA: Esto esta sección se utiliza únicamente si no
    % se comenta el ciclo "for" al definir la posición de los agentes. 

    while (t < 3)
        for i = 1:N
            E = 0;
            for j = 1:N
                V(3,i) = 1; % Velocidad de 1 para eje Z
                E = -V;
                X = X + V*dt;
                % Almacenar los datos de la velocidad durante la simulación
                historico(ciclos,:) = (sum(V.^2,1)).^0.5;
                t = t + dt;
                ciclos = ciclos + 1;
            end
        end
    end

    while(t < T)
       for i = 1:N
            E = 0;
            for j = 1:N
                dist = X(:,i)- X(:,j); % vector de distancia entre agente i y j
                mdist = norm(dist);    % norma euclidiana vector de xi - xj
                dij = d(i,j);        % distancia deseada entre agentes i y j

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
            % Implementación de límite de velocidad
            if(norm(E) > VelMax)
                E(1) = (E(1)/norm(E))*VelMax;
                E(2) = (E(2)/norm(E))*VelMax;
                E(3) = (E(3)/norm(E))*VelMax;
            end
        % Actualización de velocidad
        V(:,i) = -1*E;
        
       end
 
        % Al llegar muy cerca de la posición deseada realizar cambio de control
        if(norm(V) < 2)
            cambio = 1;
        end
        
        X = X + V*dt;

        % Almacenamiento de variables
        historico(ciclos,:) = (sum(V.^2,1)).^0.5;
        
        % Actualización de tiempo
        t = t + dt;
        ciclos = ciclos + 1;
    end
    
    %% Cálculo del error final
    mDistF = DistEntreAgentes(X);
    errorF = ErrorForm(mDistF,MatrizF(Formacion)); % error de formación simulación I
    energiaI = sum(historico.*dt,1);            % energía individual simulación I
    energiaT = sum(energiaI,2);                 % energía total simulación I
    
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
