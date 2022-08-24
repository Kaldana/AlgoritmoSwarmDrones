% =========================================================================
% MEDICI�N DE M�TRICAS EN LA SIMULACI�N DEL MODELO DIN�MICO CON CONTROL DE 
% FORMACI�N, USANDO COSENO HIPERB�LICO, Y EVASI�N DE OBST�CULOS INCLUYENDO 
% L�MITES DE VELOCIDAD
% =========================================================================
% Autor: Andrea Maybell Pe�a Echeverr�a
% �ltima modificaci�n: 27/08/2019
% (M�tricas MODELO 6)
% =========================================================================
% El siguiente script implementa la simulaci�n del modelo din�mico de
% modificaci�n a la ecuaci�n de consenso utilizando evasi�n de obst�culos y
% luego una combinaci�n de control de formaci�n con una funci�n de coseno 
% hiperb�lico para grafos m�nimamente r�gidos y evasi�n de obst�culos.
% Adem�s incluye cotas de velocidad para que no se sobrepasen los l�mites
% f�sicos de los agentes cierto n�mero de veces para determinar las m�tricas de error y c�lculos 
% de energ�a.
% =========================================================================

cantI = 100;                    % cantidad de simulaciones a realizar
EIndividual = zeros(cantI,10);  % energ�a individual por agente en cada simulaci�n
ETotal = zeros(1,cantI);        % energ�a total en cada simulaci�n
EI = zeros(1,cantI);            % error individual en cada simulaci�n
ExitoTotalF = 0;                % cantidad de formaciones 100% exitosas
Exito9F = 0;                    % cantidad de formaciones 90% exitosas
Exito8F = 0;                    % cantidad de formaciones 80% exitosas
Exito7F = 0;                    % cantidad de formaciones 70% exitosas
Fail = 0;                       % cantidad de formaciones fallidas

for I = 1:cantI
    %% Inicializaci�n del mundo
    gridsize = 20;      % tama�o del mundo
    initsize = 20;
    N = 4;             % n�mero de agentes
    dt = 0.01;          % per�odo de muestreo
    T = 20;             % tiempo final de simulaci�n

    % Inicializaci�n de posici�n de agentes
    X = initsize*rand(3,N) - initsize/2;
    % X(:,1) = [0,0];     % posici�n del lider
    Xi = X;

    % Inicializaci�n de velocidad de agentes
    % V = rand(2, N)-0.5; % aleatorio
    V = zeros(3, N); % ceros

    %% Selecci�n matriz y par�metros del sistema
    Form = 1;               % grafo seleccionado
    Rig = 1;                % rigidez formaci�n
    d = Fmatrix(Form,Rig);  % matriz de formaci�n
    r = 1;                  % radio agentes
    R = 10;                 % rango sensor
    VelMax = 10;            % velocidad m�xima
    
    %% Inicializaci�n simulaci�n
    t = 0;                      % inicializaci�n de tiempo
    ciclos = 1;                 % cuenta de la cantidad de ciclos 
    historico = zeros(100*T,N); % hist�rico de velocidades
    cambio = 0;                 % variable para el cambio de control
    
    while(t < T)
       for i = 1:N
            E = 0;
            for j = 1:N
                dist = X(:,i)- X(:,j); % vector xi - xj
                mdist = norm(dist);    % norma euclidiana vector xi - xj
                dij = 2*d(i,j);        % distancia deseada entre agentes i y j

                % Peso a�adido a la ecuaci�n de consenso
                if(mdist == 0 || mdist >= R)
                    w = 0;
                else
                    switch cambio
                        case 0              % inicio: acercar a los agentes sin chocar
                            w = (mdist - (2*(r + 0.5)))/(mdist - (r + 0.5))^2;
                        case {1,2}
                            if (dij == 0)   % si no hay arista, se usa funci�n "plana" como collision avoidance
                                w = 0.018*sinh(1.8*mdist-8.4)/mdist; 
                            else            % collision avoidance & formation control
                                w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)^2)/(mdist*(mdist - r)^2); 
                            end
                    end
                end
                % Tensi�n de aristas entre agentes
                E = E + w.*dist;
            end

            % Comparaci�n con la velocidad m�xima y ajuste
            if(norm(E) > VelMax)    
                ang = atan2(E(2),E(1));
                E(1) = VelMax*cos(ang);
                E(2) = VelMax*sin(ang);
            end
            % Actualizaci�n de velocidad
            V(:,i) = -1*E;

        end

        % Al llegar muy cerca de la posici�n deseada realizar cambio de control
        if(norm(V) < 0.2)
            cambio = cambio + 1;
        end
        % Actualizaci�n de la posici�n de los agentes
        X = X + V*dt;
        
        % Almacenamiento de variables
        historico(ciclos,:) = (sum(V.^2,1)).^0.5;
        
        % Actualizaci�n de tiempo
        t = t + dt;
        ciclos = ciclos + 1;
    end
    
    %% C�lculo del error final
    mDistF = 0.5*DistEntreAgentes(X);
    errorF = ErrorForm(mDistF,Fmatrix(Form,Rig)); % error de formaci�n simulaci�n I
    energiaI = sum(historico.*dt,1);            % energ�a individual simulaci�n I
    energiaT = sum(energiaI,2);                 % energ�a total simulaci�n I
    
    %% Porcentaje �xito formaci�n
    % Una formaci�n se considera exitosa con un error cuadr�tico medio 
    % menor a 0.05
    if(errorF > 0.05)   
        % Si la formaci�n no fue exitosa se evalua el �xito individual de 
        % de cada agente. Un agente lleg� a la posici�n deseada si tiene un
        % porcentaje de error menor al 15%.
        [errorR,cantAS] = ErrorIndividual(mDistF, Fmatrix(Form,Rig), 15);
    else
        % El que la formaci�n haya sido exitosa implica que todos los
        % agentes llegaron a la posici�n deseada
        errorR = errorF;    % error de formaci�n relativo
        cantAS = N;         % cantidad de agentes que llegan a la posici�n deseada
    end
    
    %% Porcentaje de agentes en posici�n deseada
    % Si el error de formaci�n sin tomar en cuenta a los agentes que se
    % alejaron considerablemente, es menor a 0.05 implica que hubo un
    % porcentaje de la formaci�n que s� se logr�. 
    if(errorR < 0.05)
        if(cantAS == N)                     % formaci�n 100% exitosa
            ExitoTotalF = ExitoTotalF + 1;
        elseif (cantAS == round(N*0.9))     % formaci�n 90% exitosa
            Exito9F = Exito9F + 1;
        elseif (cantAS == round(N*0.8))     % formaci�n 80% exitosa     
            Exito8F = Exito8F + 1;
        elseif (cantAS == round(N*0.7))     % formacion 70% exitosa
            Exito7F = Exito7F + 1;
        else                                % formaci�n fallida
            Fail = Fail +1;
        end
    else
        Fail = Fail + 1;
    end
        
    VResults = [ExitoTotalF, Exito9F, Exito8F, Exito7F, Fail];
end

% Guardar resultados como resulte m�s conveniente
