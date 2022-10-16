% =========================================================================
% FUNCIÓN FMATRIX
% =========================================================================
% Autor: Andrea Maybell Peña Echeverría
% Última modificación: 30/04/2019
% =========================================================================

function [d] = MatrizF(f)
%FMATRIX Retorna matriz de formación deseada según la rigidez y el número
%       de grafo
% Parámetros:
%   f = número de formación / número de grafo
%   r = nivel de rigidez (de 1 a 8)
% Salidas
%   d = matriz de adyacencia de la formación
acc = 2; %Tamano aristas del cubo
d2c = ((acc)^2 + (acc)^2)^(1/2); %Diagonal en 2D para cubo.
d3c = ((d2c)^2 + (acc)^2)^(1/2); %Diagonal en 3D para cubo.
ac = 3;
d2t = ((ac)^2 - (ac/2)^2)^(1/2); %distancia entre centro del triangulo y vertice
dpbc = ((d2t)^2 + (ac)^2)^(1/2); %distancia entre el vertice superior del triangulo y base del otro
dbb = ((ac/2)^2 + (ac)^2)^(1/2); %distancia entre el vertices inferiores del triangulo y base del otro
dbs = ((ac)^2 + (ac)^2)^(1/2); % distancia entre vertices inveriores del triangulo y vertice superior del otro
% matrices de adyacencia grafo mínimamente rígido   
    dt1 = [0    ac   ac   d2t  ac   dbs   dbs  dpbc;
           ac   0    ac   ac/2 dbs  ac    dbs  dbb;
           ac   ac   0    ac/2 dbs  dbs   ac   dbb;
           d2t  ac/2 ac/2 0    dpbc dbb   dbb  ac;
           ac   dbs  dbs  dpbc 0    ac    ac   d2t;
           dbs  ac   dbs  dbb  ac   0     ac   ac/2;
           dbs  dbs  ac   dbb  ac   ac    0    ac/2;
           dpbc dbb  dbb  ac   d2t  ac/2  ac/2 0];
      
    dc1 = [0   acc  acc  d2c acc  d2c d2c d3c;
           acc  0   d2c acc  d2c d3c acc  d2c;
           acc  d2c 0   acc  d2c acc  d3c d2c;
           d2c acc  acc  0   d3c d2c d2c acc;
           acc  d2c d2c d3c 0   acc  acc  d2c;
           d2c d3c acc  d2c acc  0   d2c acc;
           d2c acc  d3c d2c acc  d2c 0   acc;
           d3c d2c d2c acc  d2c acc  acc  0];
            
% Celdas con todas las formaciones posibles
MM = {dt1,dc1};

  % Matriz seleccionada
d = MM{f};
end

