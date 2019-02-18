# Camera Calibration with Rings Pattern and  OpenCV
El presente proyecto es una implementacion de un calibrador de camara usando patrones circulares.

# Ejecucion
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

# 1. Pipeline
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/pipeline_calibrador.png?raw=true "Title")

# 2. Vista General
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/GeneralPicture.png?raw=true "Title")

# 3. Resultados
## 3.1 Preprocesamiento, Deteccion y Seguimiento de puntos de control
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/pre-detect-track.png?raw=true "Title")

## 3.2 Correccion de distorsion
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/undistord_remap.png?raw=true "Title")

## 3.3 Tiempo y Precision
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/time-acc.png?raw=true "Title")

## 3.4 Comparacion de calibracion con otros patrones y OpenCV
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/c1-c2-accuracy.png?raw=true "Title")

## 3.5 Refinamiento de Puntos de Control
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/refinement.png?raw=true "Title")

## 3.6 Comparacion de Refinamientos
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/refinement-plot-c1.png?raw=true "Title")
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/refinement-plot-c2.png?raw=true "Title")


# Fuentes
- [1] Zhengyou Zhang,“A Flexible New Technique for Camera Calibration”,2000.
- [2] Ankur Datta, “Accurate Camera Calibration using Iterative Refinement of Control Points”, 2009.
- [3] Prakash, Camera Calibration using Adaptive Segmentation and Ellipse Fitting for Localizing Control Points”, 2012.
