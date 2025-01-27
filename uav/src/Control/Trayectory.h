#ifndef TRAYECTORY_H
#define TRAYECTORY_H

#include <vector>
#include <Eigen/Dense>
#include <Vector3D.h>
#include <iostream>
#include <cmath>

class Trayectory {
public:
    Trayectory();
    Trayectory(float diameter, float fixedZ, float resolution);
    ~Trayectory();

    // Generates the next point in the trajectory
    flair::core::Vector3Df getNextPoint();

private:
    float diameter;     // Diámetro del círculo
    float fixedZ;       // Altura fija (coordenada Z)
    float resolution;   // Resolución (distancia entre puntos)
    float radius;       // Radio del círculo
    float circumference; // Circunferencia del círculo
    size_t numPoints;   // Número total de puntos en la trayectoria
    size_t currentIndex; // Índice del punto actual                       
};

#endif // TRAYECTORY_H
