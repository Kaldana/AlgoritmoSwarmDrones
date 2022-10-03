% =========================================================================
% SIMULACIÓN DEL PROBLEMA DE FORMACIÓN EN 3D
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 20/09/2022
% Basado en: "Simulación de control de formación sin modificaciones"
% de Andrea Maybell Peña Echeverría
% (MODELO 2)
% =========================================================================
% El siguiente script implementa la simulación de la ecuación de consenso
% sin modificar.
% =========================================================================

%% Inicialización del mundo
gridsize = 15;
initsize = 10;
N = 8;
dt = 0.01;
T = 5;

% Inicialización de la posición de los agentes
X = initsize*rand(3,N);
Xi = X;

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
agents = scatter3(X(1,:),X(2,:),X(3,:),[],color,'filled');
grid minor;
xlim([-gridsize, gridsize]);
ylim([-gridsize, gridsize]);
zlim([-gridsize, gridsize]);

%% Inicialización de simulación
t = 0;
ciclos = 1;
historico = zeros(100*T,N);
hX = zeros(100*T,N);
hY = zeros(100*T,N);
hZ = zeros(100*T,N);

while(t < T)
    for i = 1:N
        E = 0;
        for j = 1:N
            dist = X(:,i)- X(:,j); % vector xi - xj
            
            w = 1;
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

figure(1);
plot(0:dt:T,historico);
xlabel('Tiempo (segundos)');
ylabel('Velocidad (u.a/segundo)');
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
