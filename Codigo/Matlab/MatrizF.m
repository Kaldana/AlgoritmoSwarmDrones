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

%% MATRICES
% matrices de adyacencia grafo mínimamente rígido
    d1 = [0 5 5 0 5 0 0 0;
          5 0 0 5 0 0 5 0;
          5 0 0 5 0 5 0 0;
          0 5 5 0 0 0 0 5;
          5 0 0 0 0 5 5 0;
          0 0 5 0 5 0 0 5;
          0 5 0 0 5 0 0 5;
          0 0 0 5 0 5 5 0];
      
    d2 = [0 1 1 0 1 0 0 0;
          1 0 0 1 0 0 1 0;
          1 0 0 1 0 1 0 0;
          0 1 1 0 0 0 0 1;
          1 0 0 0 0 1 1 0;
          0 0 1 0 1 0 0 1;
          0 1 0 0 1 0 0 1;
          0 0 0 1 0 1 1 0];
% Celdas con todas las formaciones posibles
MM = {d1,d2};

  % Matriz seleccionada
d = MM{f};
end

