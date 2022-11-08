% =========================================================================
% SIMULACIÓN DEL PROBLEMA DE FORMACIÓN EN 3D CON CONTROL DE COLISIONES Y
% VELOCIDAD
% =========================================================================
% Autor: Kenneth Andree Aldana Corado
% Última modificación: 10/16/2022
% Basado en: "Simulación de control de formación sin modificaciones"
% de Andrea Maybell Peña Echeverría
% =========================================================================
% El siguiente script implementa la simulación de la ecuación modificada
% de consenso para el caso del consenso en una formación con evasión de
% colisiones y límite de velocidad.
% =========================================================================

%% Inicialización del mundo
gridsize = 10; 
initsize = 10;
N = 8; % Definir la cantidad de agentes
dt = 0.01; % Tiempo de muestreo
T = 10; % Tiempo máximo de simulación

%% Inicialización de la posición de los agentes
X = initsize*rand(3,N);

% El siguiente ciclo sirve para colocar a todos los agentes en el plano XY
% como se realizaría en pruebas físicas, con la idea de realizar la
% simulación lo más real posible.
% for s = 1:N
%     X(3,s) = 0;
% end

% AGENTES EN PLANO ARBITRARIO

%Plano 1
%     X(1,1) = 0; X(1,2) = -4; X(1,3) = 0; X(1,4) = -4; 
%     X(1,5) = 0; X(1,6) = -4; X(1,7) = 0; X(1,8) = -4;
%     
%     X(2,1) = -4;X(2,2) = -4; X(2,3) = -2;X(2,4) = -2;
%     X(2,5) = 0; X(2,6) = 0;  X(2,7) = 2; X(2,8) = 2;
%     
%     X(3,1) = 2; X(3,2) = 2; X(3,3) = 5; X(3,4) = 5;
%     X(3,5) = 8; X(3,6) = 8; X(3,7) = 11; X(3,8) = 11;
    
%Plano 2
X(1,1) = 3; X(1,2) = 1; X(1,3) = 2; X(1,4) = 3; 
X(1,5) = 0;  X(1,6) = 1;  X(1,7) = 2;  X(1,8) = 0;

X(2,1) = 3;  X(2,2) = 1;  X(2,3) = 2; X(2,4) = 3;
X(2,5) = 0;  X(2,6) = 1;  X(2,7) = 2; X(2,8) = 0;

X(3,1) = 2;  X(3,2) = -2; X(3,3) = -2; X(3,4) = -2;
X(3,5) = 2;  X(3,6) = 2; X(3,7) = 2; X(3,8) = -2;

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
d = MatrizF(2);    % matriz de formación
r = 1;               % radio agentes
VelMax = 2;         % velocidad máxima

%% Inicialización de simulación
t = 0;
ciclos = 1; % Contador de ciclos para finalizar la formación
historico = zeros(100*T,N); % Variable que almacena la velocidad de los agentes
hX = zeros(100*T,N); % Variables para almacenamiento de posición de los agentes
hY = zeros(100*T,N);
hZ = zeros(100*T,N);
cambio = 0; % Variable para el cambio de ecuación de consenso

%% Dinámica pre formación para agentes en planos XZ.
% Este bloque permite que los drones se eleven durante 3 unidades de
% tiempo hacia arriba para que puedan salir del plano y así salir de con
% la singularidad. NOTA: Esto esta sección se utiliza únicamente si no
% se tiene comentado el ciclo for al definir la posición de los agentes. 

% while (t < 3)
%     for i = 1:N
%         E = 0;
%         for j = 1:N
%             V(3,i) = 1; % Velocidad de 1 para eje Z
%             E = -V; 
%             X = X + V*dt;
%             for a = 1:N
%                 hX(ciclos,a)= X(1,a);
%                 hY(ciclos,a)= X(2,a);
%                 hZ(ciclos,a)= X(3,a);
%             end
%             % Almacenar los datos de la velocidad durante la simulación.
%             historico(ciclos,:) = (sum(V.^2,1)).^0.5;
%     
%             % Se actualiza la gráfica, se muestra el movimiento y se
%             % incrementa el tiempo.
%             agents.XData = X(1,:);
%             agents.YData = X(2,:);
%             agents.ZData = X(3,:);
%             pause(dt);
%             t = t + dt;
%             ciclos = ciclos + 1;
%         end
%     end
% end

%% Dinámica de formación de agentes
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

% Grafico de la norma de la velocidad de los agentes
figure(1);
plot(0:dt:T,historico);
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
% scatter3(5,5,5);
hold on
scatter3(Xi(1,:),Xi(2,:),Xi(3,:),[], 'k');
scatter3(X(1,:),X(2,:),X(3,:),[], 'k', 'filled');
hold off;
