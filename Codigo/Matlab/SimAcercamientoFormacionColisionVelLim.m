% =========================================================================
% SIMULACIÓN DEL PROBLEMA DE FORMACIÓN EN 3D
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 30/08/2022
% Basado en: ""
% de Andrea Maybell Peña Echeverría
% (MODELO 1)
% =========================================================================
% El siguiente script implementa la simulación de la ecuación modificada
% de consenso para el caso del consenso en una formación.
% =========================================================================

%% Inicialización del mundo
gridsize = 10; 
initsize = 10;
N = 8; %Cantidad de agentes
dt = 0.01; %Muestreo
T = 20; %Tiempo de simulación

% Inicialización de la posición de los agentes
X = initsize*rand(3,N); 
Xi = X;

% Inicialización de la velocidad de los agentes
V = zeros(3,N);

%% Grafico de posición inicial de los agentes
%  Se utiliza distinción de color por agentes
%    Rojo:    agente 1
%    Verde:   agente 2
%    Azul:    agente 3
%    Negro:   agente 4
color = [255 0 0;
         255 0 0;
         255 0 0;
         255 0 0;
         0 255 0;
         0 255 0;
         0 255 0;
         0 255 0];    
agents = scatter3(X(1,:),X(2,:),X(3,:),[],color,'filled');
grid minor;
xlim([-gridsize, gridsize]);
ylim([-gridsize, gridsize]);
zlim([-gridsize, gridsize]);

%% Selección matriz y parámetros del sistema
d = MatrizF(1);    % matriz de formación
r = 1;               % radio agentes
R = 10;              % rango sensor
VelMax = 5;         % velocidad máxima

%% Inicialización de simulación
t = 0;
ciclos = 1;
historico = zeros(100*T,N);
hX = zeros(100*T,N);
hY = zeros(100*T,N);
hZ = zeros(100*T,N);
cambio = 0;

while(t < T)
    for i = 1:N
        E = 0;
        for j = 1:N
            dist = X(:,i)- X(:,j); % vector xi - xj
            mdist = norm(dist);    % norma euclidiana vector xi - xj
            dij = d(i,j);        % distancia deseada entre agentes i y j
            
            % Peso añadido a la ecuación de consenso
            if(mdist == 0)
                w = 0;
            else
                switch cambio
                    case 0
                        w = (mdist - (2*(r + 1)))/(mdist - (r + 1))^2; %Control de acercamiento
                    case 1
                        w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)^2)/(mdist*(mdist - r)^2);  
                end 
            end
            % Tensión de aristas entre agentes
            E = E + w.*dist;
        end
        if(norm(E) > VelMax)    
            E(1) = (E(1)/norm(E))*VelMax;
            E(2) = (E(2)/norm(E))*VelMax;
            E(3) = (E(3)/norm(E))*VelMax;
        end
        % Actualización de velocidad
        V(:,i) = -1*E;

    end
    % Al llegar muy cerca de la posición deseada del controlador, se realiza cambio de control
    if(norm(V) < 2)
        cambio = 1;
    end
    % Actualización de la posición de los agentes
    X = X + V*dt;
    
    % Almacenamiento de variables para graficar
    for a = 1:N
        hX(ciclos,a)= X(1,a);
        hY(ciclos,a)= X(2,a);
        hZ(ciclos,a)= X(3,a);
    end
    historico(ciclos,:) = (sum(V.^2,1)).^0.5;
    
    % Se actualiza la gráfica, se muestra el movimiento y se incrementa el
    % tiempo
    agents.XData = X(1,:);
    agents.YData = X(2,:);
    agents.ZData = X(3,:);
    pause(dt);
    t = t + dt;
    ciclos = ciclos + 1;
end

figure(1);
plot(0:dt:T-0.01,historico);
xlabel('Tiempo (segundos)');
ylabel('Velocidad (unidades/segundo)');
ylim([-1,inf])

% trayectorias
figure(2);
hold on;
grid on;
plot3(hX,hY,hZ,'--');
xlabel('Posición en eje X (u.a)');
ylabel('Posición en eje Y (u.a)');
zlabel('Posición en eje Z (u.a)');
scatter3(Xi(1,:),Xi(2,:),Xi(3,:),[], 'k');
scatter3(X(1,:),X(2,:),X(3,:),[], 'k', 'filled');
hold off;
