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

d3d = (75)^(1/2); %Diagonal en 3D para cubo
d2d = (25)^(1/2); %Diagonal en 2D para cubo
ac = 5; %Tamano aristas del cubo
% matrices de adyacencia grafo mínimamente rígido   
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

