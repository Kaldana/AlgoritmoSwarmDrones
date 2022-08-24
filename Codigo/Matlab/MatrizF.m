% =========================================================================
% FUNCI�N FMATRIX
% =========================================================================
% Autor: Andrea Maybell Pe�a Echeverr�a
% �ltima modificaci�n: 30/04/2019
% =========================================================================

function [d] = MatrizF(f)
%FMATRIX Retorna matriz de formaci�n deseada seg�n la rigidez y el n�mero
%       de grafo
% Par�metros:
%   f = n�mero de formaci�n / n�mero de grafo
%   r = nivel de rigidez (de 1 a 8)
% Salidas
%   d = matriz de adyacencia de la formaci�n

%% MATRICES
% matrices de adyacencia grafo m�nimamente r�gido
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

