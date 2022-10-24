# Código funcional del algoritmo de coordinación de agentes en 3D

## Funciones auxiliares
- **DistEntreAgentes.m** : en este archivo se encuentra la función que permite que los agentes sepan la posición relativa de los agentes vecinos.
- **MatriF.m** : en este archivo se encuentra la función que detalla las matrices de rigidez utilizadas para las formaciones en 3D.
- **ErrorForm.m** : en este archivo se encuentra las funciones para definir el error cuadrático de las formaciones.
- **ErrorIndividual.m** : en este archivo se encuentra las operaciones necesarias para saber cuántos agentes cumplieron con su formación y cuál es el error según la posición en la cuál debía estar.

## Códigos principales
- **SimConsensoLineal.m** : se realiza el algoritmo para la solución del problema canónico de Rendezvous, en dónde se busca el consenso en un solo punto.
- **SimFormacion.m** : en este archivo se encuentra el algoritmo de formación en 3 dimensiones para 8 agentes, en dónde se busca que cumplan la formación sin importar si colisionan o no.
- **SimAceramiento.m** : en este archivo se encuentra el algoritmo de acercamiento de drones hasta *x* unidades más que su radio.
- **SimAcercamientoFormacionColision.m** : en este archivo se encuentra el algoritmo de formación con el cuál se cuida que no hayan colisiones y se aplica formación.
- **SimAcercamientoFormacionColisionVelLim.m**: en este archivo se encuentra el algoritmo de formación en dónde se evitan colisiones y se aplica la formación con una limitante de velocidad.
- **ErrorAcercamientoFormacionColision.m**: en este archivo se encuentra la programación para definicar cuánta energía utilizaron los agentes para formarse y si la formación fue fallida o exitosa según el error cuadrático, es exitosa si es menor a 0.001 u.a.
- **ErrorAcercamientoFormacionColisionVelLim.m**: en este archivo se encuentra la programación para definicar cuánta energía utilizaron los agentes para formarse y si la formación fue fallida o exitosa según el error cuadrático, es exitosa si es menor a 0.001 u.a. Esto para el algoritmo con límite de velocidad.
