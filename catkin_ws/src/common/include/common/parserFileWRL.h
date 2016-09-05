/*
 * parserFileWRL.h
 *
 *  Created on: 06/09/2015
 *      Author: rey
 */

#ifndef PARSERFILEWRL_H_
#define PARSERFILEWRL_H_

#include "definition.h"

namespace biorobotics {

std::vector<Polygon> parserFile(std::string file) {

	std::ifstream * fileOpen = new std::ifstream();
	fileOpen->open(file.c_str());
	std::vector<Polygon> out;

	if (fileOpen->is_open()) {

		std::string line;

		while (getline(*fileOpen, line)) {

			int num_vertex = 0;
			Vertex2 * vertex = new Vertex2[num_vertex];

			if (line.length() > 0 && line.at(0) != ';') {
				if (line.at(0) == '(' && line.at(line.length() - 1) == ')') {
					ObjectType typePolygon;
					std::string shape = line.substr(1, line.length() - 2);
					std::istringstream splitM(shape);
					std::string nameLine;
					std::getline(splitM, nameLine, ' ');
					std::getline(splitM, nameLine, ' ');
					if (nameLine == "dimensions") {

					} else if (nameLine == "polygon") {
						bool flag = true;
						std::string type, name;
						do {
							std::getline(splitM, type, ' ');
						} while (type == "");
						if (type == "wall")
							typePolygon = WALL;
						else if (type == "obstacle")
							typePolygon = OBSTACLE;
						do {
							std::getline(splitM, name, ' ');
						} while (name == "");
						/*std::getline(splitM, type, ' ');
						 std::getline(splitM, name, ' ');*/
						/*std::cout << "type:" << type << std::endl;
						 std::cout << "name:" << name << std::endl;*/
						do {
							std::string xstr, ystr;
							std::getline(splitM, xstr, ' ');
							std::getline(splitM, ystr, ' ');
							if (xstr == "" || ystr == "")
								flag = false;
							else {
								vertex = (Vertex2 *) realloc(vertex,
										(++num_vertex) * sizeof(Vertex2));
								Vertex2 vert = Vertex2::Zero();
								vert.x = atof(xstr.c_str());
								vert.y = atof(ystr.c_str());
								vertex[num_vertex - 1] = vert;
							}
						} while (flag);
						Polygon polygon;
						polygon.num_vertex = num_vertex;
						polygon.objectType = typePolygon;
						polygon.vertex = vertex;
						out.push_back(polygon);
					}
				}
			}

		}
		//std::cout << "size out :" << out->size() << std::endl;
	}
	fileOpen->close();
	delete fileOpen;
	return out;
}

} /* namespace biorobotics */

#endif /* PARSERFILEWRL_H_ */
