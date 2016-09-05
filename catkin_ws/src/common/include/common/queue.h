/**
 * queue.h
 * Fecha de creación: 09/03/2016, 15:15:35
 *
 * Desarrollador Reynaldo Martell Avila
 * 2016 Universidad Nacional Autónoma de México - UNAM.
 * Instituto de Investigaciones en Matemáticas Aplicadas y en Sistemas - IIMAS.
 * Laboratorio de Bio-robótica.
 * Señales Imagenes y Ambientes Virtuales.
 */
#ifndef INCLUDE_SIMULATOR_SIMULATION_QUEUE_H_
#define INCLUDE_SIMULATOR_SIMULATION_QUEUE_H_

namespace biorobotics {

typedef struct _Node {
	int index;
	int * indexPrevious;
	int size;
} Node;

typedef struct _nodeQueue {
	Node node;
	struct _nodeQueue *next;
} node_queue;

typedef node_queue * ptrNode;
typedef node_queue * ptrQueue;

void push(ptrQueue *queue, Node node);
Node pop(ptrQueue *queue);

/*
 Agrega un nodo al inicio de la lista ligada
 *pila es el apuntador que apunta al primer nodo de la lista ligada (la cima de la pila)
 */
void push(ptrQueue *queue, Node node) {
// Crea un nuevo nodo
	ptrNode nodeQueue;
	nodeQueue = (ptrNode) malloc(sizeof(node_queue));
	if (nodeQueue != 0) {
		nodeQueue->node = node;
		// El apuntador nodo->siguiente va a apuntar al primer nodo de la lista ligada
		nodeQueue->next = *queue;
		// pila va a apuntar al nuevo nodo, con esto hacemos que el nuevo nodo sea ahora el primer nodo de la lista ligada
		*queue = nodeQueue;
	}
}

/*
 Elimina el primer nodo de la lista ligada
 *pila es el apuntador que apunta al primer nodo de la lista ligada (la cima de la pila)
 */
Node pop(ptrQueue *queue) {
// Crea un nuevo nodo
	ptrNode nodeQueue;
	Node node;

// El nuevo nodo va a apuntar al primer nodo de la lista ligada
	nodeQueue = *queue;
	node.index = (*queue)->node.index;
	node.indexPrevious = (*queue)->node.indexPrevious;
	node.size = (*queue)->node.size;
	/*punto.x = (*pila)->punto.x;
	 punto.y = (*pila)->punto.y;*/

// Ahora el segundo nodo de la lista ligada va a ser el primero
	*queue = (*queue)->next;
// Borra el primer nodo de la lista ligada
	free(nodeQueue);
// Regresa el valor que contenía el nodo que se eliminó
	return node;
}

}

#endif /* INCLUDE_SIMULATOR_SIMULATION_QUEUE_H_ */
