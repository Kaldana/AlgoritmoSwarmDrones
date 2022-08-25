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

d3d = (75)^(1/2); %Diagonal en 3D para cubo
d2d = (25)^(1/2); %Diagonal en 2D para cubo
ac = 5; %Tamano aristas del cubo
% matrices de adyacencia grafo m�nimamente r�gido   
    d1 = [0 5 5 0 5 0 0 0;
          5 0 0 5 0 0 5 0;
          5 0 0 5 0 5 0 0;
          0 5 5 0 0 0 0 5;
          5 0 0 0 0 5 5 0;
          0 0 5 0 5 0 0 5;
          0 5 0 0 5 0 0 5;
          0 0 0 5 0 5 5 0];
      
      
    dr1 = [0   ac  ac  d2d ac  d2d d2d d3d;
           ac  0   d2d ac  d2d d3d ac  d2d;
           ac  d2d 0   ac  d2d ac  d3d d2d;
           d2d ac  ac  0   d3d d2d d2d   ac;
           ac  d2d d2d d3d 0   ac  ac  d2d;
           d2d d3d ac  d2d ac  0   d2d ac;
           d2d ac  d3d d2d ac  d2d 0   ac;
           d3d d2d d2d ac  d2d   ac  ac  0];
% Celdas con todas las formaciones posibles
MM = {d1,dr1};

  % Matriz seleccionada
d = MM{f};
end

