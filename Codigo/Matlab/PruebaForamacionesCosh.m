% =========================================================================
% SIMULACIÓN DEL PROBLEMA DE RENDEZVOUS EN 3D
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 08/09/2022
% Basado en: "Simulación de control de formación sin modificaciones"
% de Andrea Maybell Peña Echeverría
% (MODELO 0)
% =========================================================================
% El siguiente script implementa la simulación de la ecuación 
% de consenso para el caso del consenso en un punto.
% =========================================================================

%% Inicialización del mundo
gridsize = 20;
initsize = 20;
N = 8;
dt = 0.01;
T = 20;

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
d = MatrizF(2);    % matriz de formación
r = 5;               % radio agentes
R = 20;              % rango sensor
VelMax = 10;         % velocidad máxima

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
            dij = 2*d(i,j);        % distancia deseada entre agentes i y j
            
            % Peso añadido a la ecuación de consenso
            if(mdist == 0 || mdist >= R)
                w = 0;
            else
                switch cambio
                    case 0              % inicio: acercar a los agentes sin chocar
                        w = (mdist - (2*(r + 0.5)))/(mdist - (r + 0.5))^2;
                    case {1,2}
                        if (dij == 0)   % si no hay arista, se usa función "plana" como collision avoidance
                            w = 0.018*sinh(1.8*mdist-8.4)/mdist; 
                        else            % collision avoidance & formation control
                            w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)^2)/(mdist*(mdist - r)^2); 
                        end
                end
            end
            % Tensión de aristas entre agentes
            E = E + w.*dist;
        end
        % Comparación con la velocidad máxima y ajuste
        if(norm(E) > VelMax)    
            ang = atan2(E(2),E(1));
            E(1) = VelMax*cos(ang);
            E(2) = VelMax*sin(ang);
        end
        
        % Actualización de velocidad
        V(:,i) = -1*E;

    end

    % Al llegar muy cerca de la posición deseada realizar cambio de control
    if(norm(V) < 0.2)
        cambio = cambio + 1;
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
plot3(hX,hY,hZ,'--');
xlabel('Posición en eje X (unidades)');
ylabel('Posición en eje Y (unidades)');
zlabel('Posición en eje Z (unidades)');
scatter3(Xi(1,:),Xi(2,:),Xi(3,:),[], 'k');
scatter3(X(1,:),X(2,:),X(3,:),[], 'k', 'filled');
hold off;
