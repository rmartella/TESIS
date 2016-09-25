/**
 * PotentialFields.h
 * Fecha de creación: 22/02/2016, 15:08:54
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Seńales Imagenes y Ambientes Virtuales.
 */
#ifndef SRC_SIMULATION_POTENTIALFIELDS_H_
#define SRC_SIMULATION_POTENTIALFIELDS_H_

 #include <common/utilSimulator.h>

 using namespace biorobotics;

class PotentialFields {
public:
    PotentialFields(float etha, float dr, float da, float epsilon1,
            float epsilon2);
    virtual ~PotentialFields();
    Vertex2 computeTotalForze(Vertex2 position,
            std::vector<Polygon> polygons, Vertex2 goal);
    Vertex2 computeTotalForzeWithSensors(Vertex2 position, float currTheta,
            LaserScan * laserScan,
            Vertex2 goal);

protected:
    Vertex2 computeGradientRepulsionObject(Vertex2 position,
            Vertex2 obstaclePosition, float dr);
    Vertex2 computeRepulsionForze(Vertex2 position,
            std::vector<Polygon> polygons);
    Vertex2 computeRepulsionForzeWithSensors(Vertex2 position, float currTheta,
            LaserScan * laserScan);
    Vertex2 computeAtractionForze(Vertex2 position,
            Vertex2 goal);

private:
    float etha;
    float dr;
    float da;
    float epsilon1;
    float epsilon2;
};

#endif /* SRC_SIMULATION_POTENTIALFIELDS_H_ */