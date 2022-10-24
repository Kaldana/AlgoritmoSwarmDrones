% =========================================================================
% SIMULACIÓN DE ACERCAMIENTO DE AGENTES
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 10/15/2022
% Basado en: "Simulación de control de formación sin modificaciones"
% de Andrea Maybell Peña Echeverría
% =========================================================================
% El siguiente script implementa la simulación de la ecuación modificada
% de consenso para el caso de acercamiento.
% =========================================================================

%% Inicialización del mundo
gridsize = 20; 
initsize = 15; 
N = 8; % Definir la cantidad de agentes
dt = 0.01; % Tiempo de muestreo
T = 20; % Tiempo máximo de simulación

% Inicialización de la posición de los agentes
X = initsize*rand(3,N);
Xi = X; % Vector de posición de los agentes

% Inicialización de la velocidad de los agentes
V = zeros(3,N);

%% Grafico de posición inicial de los agentes
%  Se utiliza distinción de color por agentes
%    Rojo:    agente 1
%    Verde:   agente 2

color = [255 0 0;
         255 0 0;
         255 0 0;
         255 0 0;
         0 255 0;
         0 255 0;
         0 255 0;
         0 255 0];    
% Se define la representación gráfica como gráfico de dispersión en 3D
agents = scatter3(X(1,:),X(2,:),X(3,:),[],color,'filled');
grid minor;
xlim([-gridsize, gridsize]);
ylim([-gridsize, gridsize]);
zlim([-gridsize, gridsize]);

%% Selección matriz y parámetros del sistema
d = MatrizF(1);    % matriz de formación
r = 1;               % radio agentes

%% Inicialización de simulación
t = 0;
ciclos = 1; % Contador de ciclos para finalizar la formación
historico = zeros(100*T,N); % Variable que almacena la velocidad de los agentes
hX = zeros(100*T,N); % Variables para almacenamiento de posición de los agentes
hY = zeros(100*T,N);
hZ = zeros(100*T,N);


while(t < T)
    for i = 1:N
        E = 0;
        for j = 1:N
            dist = X(:,i)- X(:,j); % vector de distancia entre agente i y j 
            mdist = norm(dist);    % norma euclidiana vector xi - xj
            dij = d(i,j);        % distancia deseada entre agentes i y j
            
            % Se calcula el peso para la ecuación
            if(mdist == 0)
                w = 0;
            else
                w = (mdist - (2*(r)))/(mdist - (r))^2; % ecuación de consenso modificada para acercamiento
            end
            
            % Tensión de aristas entre agentes
            E = E + w.*dist;
        end
       
        % Actualización de velocidad
        V(:,i) = -1*E;

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

% Grafico de la norma de la velocidad de los agentes
figure(1);
plot(0:dt:T-0.01,historico);
xlabel('Tiempo (segundos)');
ylabel('Velocidad (unidades/segundo)');
ylim([-1,inf])

% Grafico de posición inicial y final de los agentes, adicionalmente se
% superpone su trayectoria.
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
