#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <stdio.h>
#include <vector>
#include <glm/glm.hpp>

bool loadOBJ(
	const char * path, 
	std::vector<float> & out_data,
	int * vertexnum
);

bool loadOBJwithcolor(
	const char * path, 
	std::vector<float> & out_data,
	int * vertexnum,
	glm::vec3 colorbar[]
);

bool loadMesh(
	FILE* file, 
	std::vector<float> & out_data,
	glm::vec3 const & color,
	int *vertexnum);

bool loadAssImp(
	const char * path, 
	std::vector<unsigned short> & indices,
	std::vector<glm::vec3> & vertices,
	std::vector<glm::vec2> & uvs,
	std::vector<glm::vec3> & normals
);


#endif