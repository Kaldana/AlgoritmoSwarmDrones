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
- **ErrorAcercamientoFormacionColision.m**: en este archivo se encuentra la programación para definicar cuánta energía utilizaron los agentes para formarse y si la formación fue fallida o exitosa según el error cuadrático, es exitosa si es menor a 0.001 u.a. Adicionalmente, se incorpora dos secciones para que la inicialización de los agentes sea en dos planos distintos y también se agrega un bloque para dar una dinámica inicial a los agentes y así puedan romper el plano inicial de Z = 0.
- **ErrorAcercamientoFormacionColisionVelLim.m**: en este archivo se encuentra la programación para definicar cuánta energía utilizaron los agentes para formarse y si la formación fue fallida o exitosa según el error cuadrático, es exitosa si es menor a 0.001 u.a. Esto para el algoritmo con límite de velocidad. Adicionalmente, se incorpora dos secciones para que la inicialización de los agentes sea en dos planos distintos y también se agrega un bloque para dar una dinámica inicial a los agentes y así puedan romper el plano inicial de Z = 0.

## Uso de códigos principales
- **SimConsensoLineal.m** : para este código basta con darle "Run" en matlab, se realizará el consenso según la cantidad de agentes indicados en la variable N.
- **SimFormacion.m** : para seleccionar la formación se debe variar el número ingresado en la función auxiliar MatriF, 1 para prisma triangular y 2 para cubo. Posteriormente presionar "Run" para que se ejecute la simulación.
- **SimAcercamiento.m** : para seleccionar la formación se debe variar el número ingresado en la función auxiliar MatriF, 1 para prisma triangular y 2 para cubo. Posteriormente presionar "Run" para que se ejecute la simulación.
- **SimAcercamientoFormacionColision.m** : para seleccionar la formación se debe variar el número ingresado en la función auxiliar MatriF, 1 para prisma triangular y 2 para cubo. Posteriormente presionar "Run" para que se ejecute la simulación.
- **SimAcercamientoFormacionColisionVelLim.m**: para seleccionar la formación se debe variar el número ingresado en la función auxiliar MatriF, 1 para prisma triangular y 2 para cubo. Posteriormente presionar "Run" para que se ejecute la simulación. En este código se implementan dos tipos de simulaciones. La primera son simulaciones en planos iniciando la formación con todos los drones en el suelo y la segunda es en planos pero con rutina para que los drones se eleven para posteriormente realizar la formación. Si se desea iniciar en planos, descomentar las secciones de Plano 1 o Plano 2. Si se desea que todos inicien en el suelo, descomentar la sección de "for s= 1:N ... end".  Para implementar la rutina de elevación previo a formación, descomentar la sección de "Dinámica pre formación para agentes en planos XZ".