#ifndef VISIONDRONE
#define VISIONDRONE

#include <set>
#include <stdio.h>
#include <vector>
#include <glm/glm.hpp>

typedef glm::i32vec4 triunit;

enum cubedirect
{
	left,right,
	top,bottom,
	front,back
};

typedef struct
{
	std::set<int> vertice;
	int normalind;
	//enum cubedirect dirct;
	glm::vec3 normal;
} rectunit;


class cubecoord
{
private:
    glm::vec3 vertices[8];
	glm::vec3 front;
	glm::vec3 right;
	glm::vec3 top;

	glm::mat3 Rotation;
	glm::vec3 neworigin;

	float divideconst[6];

public:
	cubecoord(std::vector<triunit> & cube, 	std::vector<glm::vec3> & temp_vertices, std::vector<glm::vec3> & temp_normals);
	cubecoord();
	~cubecoord();
	std::vector<glm::vec3> contourpoint(glm::vec3 campos);
};

bool loadcubes( const char * path, std::vector<cubecoord> & cubecontours);


#endif