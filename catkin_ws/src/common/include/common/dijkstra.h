/**
 * dijkstra.h
 * Fecha de creación: 17/03/2016, 0:42:03
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef INCLUDE_SIMULATOR_SIMULATION_DIJKSTRA_H_
#define INCLUDE_SIMULATOR_SIMULATION_DIJKSTRA_H_

#include "queue.h"
#include "definition.h"

#define IN 9999

namespace biorobotics {

Node dijsktra(bool ** adyacencies, int sizeAdyacencies, Vertex2 * vertexMap);

/*float ** createCost(Vertex inicio, Vertex fin, int num_polygons,
 Polygon * polygons, Polygon * grownPolygons, bool ** adyacencias) {*/
float ** createCost(bool ** adyacencies, int sizeAdyacencies,
		Vertex2 * vertexMap) {
	float ** costos = (float **) malloc(sizeAdyacencies * sizeof(float *));

	for (int i = 0; i < sizeAdyacencies; i++) {
		(costos[i] = (float *) malloc(sizeAdyacencies * sizeof(float)));
	}
	for (int i = 0; i < sizeAdyacencies; i++) {
		for (int j = 0; j < sizeAdyacencies; j++)
			costos[i][j] = IN;
	}
	for (int i = 0; i < sizeAdyacencies; i++) {
		for (int j = 0; j < sizeAdyacencies; j++) {
			if (adyacencies[i][j]) {
				Vertex2 verice1 = vertexMap[i];
				Vertex2 verice2 = vertexMap[j];
				costos[i][j] = verice1.sub(verice2).norm();
			}
		}
	}
	return costos;
}

Node dijsktra(bool ** adyacencies, int sizeAdyacencies, Vertex2 * vertexMap) {
	float ** cost = createCost(adyacencies, sizeAdyacencies, vertexMap);

	int prev[sizeAdyacencies], selected[sizeAdyacencies], i, m, start, j;
	float dist[sizeAdyacencies], min, d;
	for (i = 0; i < sizeAdyacencies; i++) {
		dist[i] = IN;
		prev[i] = -1;
		selected[i] = 0;
	}
	start = sizeAdyacencies - 2;
	selected[start] = 1;
	dist[start] = 0;
	while (selected[sizeAdyacencies - 1] == 0) {
		min = IN;
		m = -1;
		for (i = 0; i < sizeAdyacencies; i++) {
			d = dist[start] + cost[start][i];
			if (d < dist[i] && selected[i] == 0) {
				dist[i] = d;
				prev[i] = start;
			}
			if (min > dist[i] && selected[i] == 0) {
				min = dist[i];
				m = i;
			}
		}
		start = m;
		selected[start] = 1;
	}
	start = sizeAdyacencies - 1;
	int * previousVertexTmp = (int *) malloc(sizeof(int));
	j = 0;
	while (start != -1) {
		previousVertexTmp = (int *) realloc(previousVertexTmp,
				sizeof(int) * (j + 1));
		previousVertexTmp[j] = start;
		j++;
		start = prev[start];
	}
	int * previousVertex = (int *) malloc(sizeof(int) * (j - 1));
	for (int i = 0; i < j - 1; i++) {
		previousVertex[i] = previousVertexTmp[j - i - 1];
	}
	free(previousVertexTmp);
	free(cost);

	Node nodePath = { sizeAdyacencies - 1, previousVertex, j - 1 };
	return nodePath;

}

}

#endif /* INCLUDE_SIMULATOR_SIMULATION_DIJKSTRA_H_ */
