/**
 * PotentialFields.cpp
 * Fecha de creación: 22/02/2016, 15:08:54
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Seńales Imagenes y Ambientes Virtuales.
 */

#include <ros/ros.h>
#include <navigation/PotentialFields.h>

PotentialFields::PotentialFields(float etha, float dr, float da, float epsilon1,
        float epsilon2) {
    this->etha = etha;
    this->dr = dr;
    this->da = da;
    this->epsilon1 = epsilon1;
    this->epsilon2 = epsilon2;
}

PotentialFields::~PotentialFields() {
    // TODO Auto-generated destructor stub
}

Vertex2 PotentialFields::computeGradientRepulsionObject(Vertex2 position,
        Vertex2 obstaclePosition, float dr) {
    float module = position.sub(obstaclePosition).norm();
    float x, y, scalar;
    Vertex2 grad = Vertex2::Zero();
    if (module <= dr) {
        x = position.x - obstaclePosition.x;
        y = position.y - obstaclePosition.y;
        scalar = 1 / module - 1 / dr;
        //scalar = (-etha * scalar * (1 / pow(module, 3)));
        scalar = (scalar * (1 / pow(module, 3)));
        x = scalar * x;
        y = scalar * y;
        grad.x = x;
        grad.y = y;
    } else {
        grad.x = 0;
        grad.y = 0;
    }
    return grad;
}

Vertex2 PotentialFields::computeRepulsionForze(Vertex2 position,
        std::vector<Polygon> polygons) {
    std::vector<Vertex2> centroids = computeCentroids(polygons);
    Vertex2 totalForze = Vertex2::Zero();
    for (unsigned int i = 0; i < polygons.size(); i++) {
        Vertex2 forzeObj = computeGradientRepulsionObject(position,
                centroids.at(i), dr);
        totalForze.x = totalForze.x + forzeObj.x;
        totalForze.y = totalForze.y + forzeObj.y;
    }
    return totalForze;
}

Vertex2 PotentialFields::computeRepulsionForzeWithSensors(Vertex2 position, float currTheta,
        LaserScan * laserScan) {
    float inc_angle;
    Vertex2 totalForze = Vertex2::Zero();
    for (unsigned int i = 0; i < laserScan->num_scans; i++) {
        if(laserScan->ranges[i] >= laserScan->range_min && laserScan->ranges[i] <= laserScan->range_max){
            Vertex2 obst = Vertex2::Zero();
            obst.x = position.x + laserScan->ranges[i] * cos(currTheta + laserScan->angle_min + i * laserScan->angle_increment);
            obst.y = position.y + laserScan->ranges[i] * sin(currTheta + laserScan->angle_min + i * laserScan->angle_increment);
            Vertex2 forze = computeGradientRepulsionObject(position, obst,
                        dr);
            totalForze.x = totalForze.x + forze.x;
            totalForze.y = totalForze.y + forze.y;    
        }
    }
    totalForze.x = -etha / laserScan->num_scans * totalForze.x;
    totalForze.y = -etha / laserScan->num_scans * totalForze.y;
    return totalForze;
}

Vertex2 PotentialFields::computeAtractionForze(Vertex2 position,
        Vertex2 goal) {
    float module = position.sub(goal).norm();
    Vertex2 atractionForze = Vertex2::Zero();
    if (module <= da) {
        atractionForze.x = epsilon1 * (position.x - goal.x);
        atractionForze.y = epsilon1 * (position.y - goal.y);
    } else {
        atractionForze.x = epsilon2 / module
                * (position.x - goal.x);
        atractionForze.y = epsilon2 / module
                * (position.y - goal.y);
    }
    return atractionForze;
}

Vertex2 PotentialFields::computeTotalForze(Vertex2 position,
        std::vector<Polygon> polygons, Vertex2 goal) {

    Vertex2 totalForze = Vertex2::Zero();
    Vertex2 atractionForze = computeAtractionForze(position,
            goal);
    Vertex2 repulsionForze = computeRepulsionForze(position, polygons);
    totalForze.x = atractionForze.x + repulsionForze.x;
    totalForze.y = atractionForze.y + repulsionForze.y;
    ROS_INFO("atractionForze Fa(%f,%f)", atractionForze.x, atractionForze.y);
    ROS_INFO("repulsionForze Fr(%f,%f)", repulsionForze.x, repulsionForze.y);
    return totalForze;
}

Vertex2 PotentialFields::computeTotalForzeWithSensors(Vertex2 laserPosition, float currTheta,
        LaserScan * laserScan, Vertex2 goal) {
    Vertex2 totalForze = Vertex2::Zero();
    Vertex2 atractionForze = computeAtractionForze(laserPosition,
            goal);
    Vertex2 repulsionForze = computeRepulsionForzeWithSensors(laserPosition, currTheta,
            laserScan);
    totalForze.x = atractionForze.x + repulsionForze.x;
    totalForze.y = atractionForze.y + repulsionForze.y;
    //ROS_INFO("atractionForze Fa(%f,%f)", atractionForze.x, atractionForze.y);
    //ROS_INFO("repulsionForze Fr(%f,%f)", repulsionForze.x, repulsionForze.y);
    return totalForze;
}