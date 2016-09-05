/**
 * first_depth.h
 * Fecha de creación: 09/03/2016, 15:33:08
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef INCLUDE_SIMULATOR_SIMULATION_FIRST_DEPTH_H_
#define INCLUDE_SIMULATOR_SIMULATION_FIRST_DEPTH_H_

#include "queue.h"
#include "definition.h"

namespace biorobotics {

Node firstDepth(bool ** adyacencies, int sizeAdyacencies, Vertex2 * vertexMap);
void addNodesToExpantion(Node node, bool ** adyacencias, int sizeAdjacencies,
		Node * expantion, ptrQueue * queue);
bool checkExpantion(Node node, Node * expantion, int indexExpantion);

Node firstDepth(bool ** adyacencies, int sizeAdyacencies, Vertex2 * vertexMap) {

	int indexExpantion = 0;
	int indexEnd = sizeAdyacencies - 1;
	Node * expantion = (Node *) malloc(sizeof(Node));

	int * previousIndexPtr = (int *) malloc(sizeof(int));
	*previousIndexPtr = -1;
	Node node = Node { sizeAdyacencies - 2, 0, 0 };
	ptrQueue queue = 0;
	expantion[0] = node;
	indexExpantion = 1;
	push(&queue, node);
	node = pop(&queue);
	while (node.index != indexEnd) {
		addNodesToExpantion(node, adyacencies, sizeAdyacencies, expantion,
				&queue);
		node = pop(&queue);
		while (!checkExpantion(node, expantion, indexExpantion)) {
			node = pop(&queue);
		}
		expantion = (Node *) realloc(expantion,
				sizeof(Node) * (indexExpantion + 1));
		expantion[indexExpantion] = node;
		indexExpantion = indexExpantion + 1;
	}
	return node;
}

void addNodesToExpantion(Node node, bool ** adyacencias, int sizeAdjacencies,
		Node * expantion, ptrQueue * queue) {
	bool * columToExpantion;
	int index = node.index;

	columToExpantion = adyacencias[index];
	for (int j = 0; j < sizeAdjacencies; j++) {
		if (columToExpantion[j]) {
			int * indexPrevious = (int *) malloc(sizeof(int) * (node.size + 1));
			indexPrevious[node.size] = index;
			if (node.indexPrevious != 0)
				memcpy(indexPrevious, node.indexPrevious,
						node.size * sizeof(int));
			Node newNode = { j, indexPrevious, node.size + 1 };
			push(queue, newNode);
		}
	}
}

bool checkExpantion(Node node, Node * expantion, int indexExpantion) {
	bool flagExpantion = true;
	for (int i = 0; i < indexExpantion && flagExpantion; i++) {
		if (node.index == expantion[i].index) {
			flagExpantion = false;
		}
	}
	return flagExpantion;
}

}

#endif /* INCLUDE_SIMULATOR_SIMULATION_FIRST_DEPTH_H_ */
