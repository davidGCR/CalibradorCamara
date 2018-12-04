# CalibradorCamara
El presente proyecto es una implementacion de un calibrador de camara usando patrones circulares.

# Ejecucinn
Requisitos:
- Sistema Operativo: Ubuntu
- Opencv 3.2.0 o superior
- Compilador gcc, g++

Ejecucion:
Para ejecutar el proyecto una ves que tenga los requisitos siga los siguientes pasos:
- Clone o descargue el proyecto.
- Abra un terminal y dirijase a la carpeta del proyecto que clono.
- Compile con el siguiente comando:
g++ -ggdb `pkg-config --cflags opencv` -o `basename test1.cpp .cpp` test1.cpp `pkg-config --libs opencv`
- Ejecute la aplicacion con el comando:
./test1

# Fuentes
- [1] Zhengyou Zhang,“A Flexible New Technique for Camera Calibration”,2000.
- [2] Ankur Datta, “Accurate Camera Calibration using Iterative Refinement of Control Points”, 2009.
- [3] Prakash, C¸ amera Calibration using Adaptive Segmentation and Ellipse Fitting for Localizing Control Points”, 2012.
