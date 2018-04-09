#include <vector>
#include <stdio.h>
#include <string>
#include <cstring>
#include <map>

#include <cmath>
#include <glm/glm.hpp>

#include "visiondrone.hpp"

// Very, VERY simple OBJ loader.
// Here is a short list of features a real function would provide : 
// - Binary files. Reading a model should be just a few memcpy's away, not parsing a file at runtime. In short : OBJ is not very great.
// - Animations & bones (includes bones weights)
// - Multiple UVs
// - All attributes should be optional, not "forced"
// - More stable. Change a line in the OBJ file and it crashes.
// - More secure. Change another line and you can inject code.
// - Loading from memory, stream, etc

int projcase[43][7] = // [caseindx]: pointnum, point1, point2, ....
{
	{0},
	{4,0,4,7,3},
	{4,1,2,6,5},
	{0},
	{4,0,1,5,4},
	{6,0,1,5,4,7,3},
	{6,0,1,2,6,5,4},
	{0},
	{4,2,3,7,6},
	{6,4,7,6,2,3,0},
	{6,2,3,7,6,5,1},
	{0},
	{0},
	{0},
	{0},
	{0},
	{4,0,3,2,1},
	{6,0,4,7,3,2,1},
	{6,0,3,2,6,5,1},
	{0},
	{6,0,3,2,1,5,4},
	{6,2,1,5,4,7,3},
	{6,0,3,2,6,5,4},
	{0},
	{6,0,3,7,6,2,1},
	{6,0,4,7,6,2,1},
	{6,0,3,7,6,5,1},
	{0},
	{0},
	{0},
	{0},
	{0},
	{4,4,5,6,7},
	{6,4,5,6,7,3,0},
	{6,1,2,6,7,4,5},
	{0},
	{6,0,1,5,6,7,4},
	{6,0,1,5,6,7,3},
	{6,0,1,2,6,7,4},
	{0},
	{6,2,3,7,4,5,6},
	{6,0,4,5,6,2,3},
	{6,1,2,3,7,4,5}
};


int findcommon(std::set<int> rect1, std::set<int> rect2, std::set<int> rect3)
{
	for(auto it = rect1.begin(); it!= rect1.end(); it++)
	{
		if(rect2.count(*it)!= 0 && rect3.count(*it)!=0)
			return *it;
	}

	return -1;
}
cubecoord::cubecoord(){}
cubecoord::~cubecoord(){}

cubecoord::cubecoord(std::vector<triunit> & cube, 	std::vector<glm::vec3> & temp_vertices, std::vector<glm::vec3> & temp_normals)
{


	if(cube.size()!= 12) printf("getcubecoor: something wrong with cube unit\n");

	
	//rectunit rectangles[6];
	cubedirect directs[6];//{cubedirect::top, cubedirect::back,cubedirect::bottom,cubedirect::front, cubedirect::left, cubedirect::right};
	
	
	directs[0] = cubedirect::front;
	glm::vec3 frontdirect = temp_normals[cube[0][3]];
	glm::vec3 rightdirect;
	glm::vec3 topdirct;
	glm::vec3 normtmp = temp_normals[cube[1][3]]; 

	if( fabs(glm::dot(frontdirect,normtmp)- (-1)) < 1e-5) //parallel --> bottom
	{
		directs[1] = cubedirect::back;
		directs[2] = cubedirect::right;
		rightdirect = temp_normals[cube[2][3]];
	}
	else
	{
		directs[1] = cubedirect::right;
		rightdirect = normtmp;
	}

	topdirct = glm::cross(frontdirect,rightdirect);
	
	int dirctind;
	
	for(dirctind = 0; dirctind < 6; dirctind++)   //find top
	{
		normtmp = temp_normals[cube[dirctind][3]];
		if( fabs(glm::dot(topdirct,normtmp)- (1)) < 1e-4) break;
	}
	directs[dirctind] = cubedirect::top;

	for(dirctind = 0; dirctind < 6; dirctind++)   //find backdirct
	{
		normtmp = temp_normals[cube[dirctind][3]];
		if( fabs(glm::dot(frontdirect,normtmp)- (-1)) < 1e-4) break;
	}
	directs[dirctind] = cubedirect::back;

	for(dirctind = 0; dirctind < 6; dirctind++)   //find backdirct
	{
		normtmp = temp_normals[cube[dirctind][3]];
		if( fabs(glm::dot(rightdirect,normtmp)- (-1)) < 1e-4) break;
	}
	directs[dirctind] = cubedirect::left;
	
	for(dirctind = 0; dirctind < 6; dirctind++)   //find backdirct
	{
		normtmp = temp_normals[cube[dirctind][3]];
		if( fabs(glm::dot(topdirct,normtmp)- (-1)) < 1e-4) break;
	}
	directs[dirctind] = cubedirect::bottom;

	
	
	
	
	
	
	std::map<enum cubedirect, rectunit> rectangles;

	

	for(int i = 0; i < 6; i ++)
	{
		//rectangles[i].dirct = directs[i];
		rectunit tmp;
		
		tmp.normalind = cube[i][3];
		tmp.normal = temp_normals[cube[i][3]];
		tmp.vertice.insert(&(cube[i][0]),&(cube[i][3]));
		tmp.vertice.insert(&(cube[i+6][0]),&(cube[i+6][3]));

		rectangles.insert(std::pair<enum cubedirect, rectunit>(directs[i],tmp));
	}
	



	glm::vec3 left = rectangles[cubedirect::left].normal;
	glm::vec3 bottom = rectangles[cubedirect::bottom].normal;
	glm::vec3 back = rectangles[cubedirect::back].normal;
	glm::vec3 correctbottom = glm::cross(left,back);
	float err = glm::dot(correctbottom,bottom);
	if(fabs(err-1) > 1e-4)
	{printf("warning: error normal scheme!!!!!!!\n");}

	int vertexind;
	//cubecoord out;
	front = rectangles[cubedirect::front].normal;
	right = rectangles[cubedirect::right].normal;
	top = rectangles[cubedirect::top].normal;

	// 0 --> front, left, bottom   
	vertexind = findcommon(rectangles[cubedirect::front].vertice,rectangles[cubedirect::left].vertice,rectangles[cubedirect::bottom].vertice);
	if(vertexind == -1) printf("err in find vertex 0\n");
	vertices[0] = temp_vertices[vertexind];
	// 1 --> front, right,bottom
	vertexind = findcommon(rectangles[cubedirect::front].vertice,rectangles[cubedirect::right].vertice,rectangles[cubedirect::bottom].vertice);
	if(vertexind == -1) printf("err in find vertex 1\n");
	vertices[1] = temp_vertices[vertexind];
	// 2 --> front, right,top
	vertexind = findcommon(rectangles[cubedirect::front].vertice,rectangles[cubedirect::right].vertice,rectangles[cubedirect::top].vertice);
	if(vertexind == -1) printf("err in find vertex 2\n");
	vertices[2] = temp_vertices[vertexind];
	// 3 --> front, left, top
	vertexind = findcommon(rectangles[cubedirect::front].vertice,rectangles[cubedirect::left].vertice,rectangles[cubedirect::top].vertice);
	if(vertexind == -1) printf("err in find vertex 3\n");
	vertices[3] = temp_vertices[vertexind];
	// 4 --> back, left,bottom
	vertexind = findcommon(rectangles[cubedirect::back].vertice,rectangles[cubedirect::left].vertice,rectangles[cubedirect::bottom].vertice);
	if(vertexind == -1) printf("err in find vertex 4\n");
	vertices[4] = temp_vertices[vertexind];
	// 5 --> back, right,bottom
	vertexind = findcommon(rectangles[cubedirect::back].vertice,rectangles[cubedirect::right].vertice,rectangles[cubedirect::bottom].vertice);
	if(vertexind == -1) printf("err in find vertex 5\n");
	vertices[5] = temp_vertices[vertexind];
	// 6 --> back,right,top
	vertexind = findcommon(rectangles[cubedirect::back].vertice,rectangles[cubedirect::right].vertice,rectangles[cubedirect::top].vertice);
	if(vertexind == -1) printf("err in find vertex 6\n");
	vertices[6] = temp_vertices[vertexind];
	// 7 --> back,left,top
	vertexind = findcommon(rectangles[cubedirect::back].vertice,rectangles[cubedirect::left].vertice,rectangles[cubedirect::top].vertice);
	if(vertexind == -1) printf("err in find vertex 7\n");
	vertices[7] = temp_vertices[vertexind];

	

	neworigin = glm::vec3(0,0,0);
	for(int i = 0; i < 8; i ++)
	{
//		printf("%f, %f, %f\n",cube.vertices[i][0],cube.vertices[i][1],cube.vertices[i][2]);
		neworigin += vertices[i];
		
	}
	neworigin /= 8;

	glm::mat3 R;
	R[0][0] = front[0]; R[0][1] = front[1]; R[0][2] = front[2]; 
	R[1][0] = right[0]; R[1][1] = right[1]; R[1][2] = right[2];
	R[2][0] = top[0];   R[2][1] = top[1];   R[2][2] = top[2];   


	Rotation = glm::inverse(R);

	//for test
	glm::vec3 verticeprime[3];
	for(int i = 0; i < 3; i ++)
	{
		auto tmp = vertices[i];
		tmp = tmp - neworigin;
		tmp = Rotation*tmp;
		verticeprime[i] = tmp;
		//printf("%f, %f, %f\n",tmp[0],tmp[1],tmp[2]);
	}

	//divideconst[6] : [-b,b,-c,c,a,-a]
	//compute right : y = b x1[1]
	//compute left  : y = -b x0[1]
	divideconst[0] = -verticeprime[1][1] -1e-6;
	divideconst[1] = verticeprime[1][1]  +1e-6;

	//compute top  :  z = c x2[2]
	//compute bottom: z = -c x1[2]
	divideconst[2] = -verticeprime[2][2] - 1e-6;
	divideconst[3] = verticeprime[2][2]  + 1e-6;
	
	//compute front : x = a  x0[0]
	//compute back  : x = -1 x4[0]
	divideconst[4] = verticeprime[0][0]  + 1e-6;
	divideconst[5] = -verticeprime[0][0] - 1e-6;
	

}

std::vector<glm::vec3> cubecoord::contourpoint(glm::vec3 campos)
{
	//case code: 5_4_3_2_1_0: back front top bottom right left
	
	glm::vec3 transformedcam = Rotation*(campos - neworigin);
	int caseindx = 0;
	

	caseindx += (1)*(transformedcam[1] < divideconst[0] ? 1:0);    //-b
	caseindx += (2)*(transformedcam[1] > divideconst[1] ? 1:0);    // b
	caseindx += (4)*(transformedcam[2] < divideconst[2] ? 1:0);    //-c
	caseindx += (8)*(transformedcam[2] > divideconst[3] ? 1:0);    // c 
	caseindx += (16)*(transformedcam[0] > divideconst[4] ? 1:0);   //-a 
	caseindx += (32)*(transformedcam[0] < divideconst[5] ? 1:0);   // a

	std::vector<int> verticeind;
	if(caseindx == 0) printf("inside the cube\n");
	else if(caseindx >= 43) printf("wrong with projection computation\n");
	else if(projcase[caseindx][0] == 0) printf("wrong with projection computation\n");
	else
	{
		for(int i = 0; i < projcase[caseindx][0];i++ )
			verticeind.push_back(projcase[caseindx][1 + i]);
	}

	std::vector<glm::vec3> contour;
	for(auto verind:verticeind)
	{
		contour.push_back(vertices[verind]);
	}

	return std::move(contour);
}

bool loadcubes( const char * path, std::vector<cubecoord> & cubecontours)
{
	//printf("Loading OBJ file %s...\n", path);

	std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
	std::vector<glm::vec3> temp_vertices; 
	std::vector<glm::vec3> temp_uvs;
	std::vector<glm::vec3> temp_normals;
	
	std::vector<std::vector<triunit>> cubes(5);

	FILE * file = fopen(path, "r");
	if( file == NULL ){
		printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
		getchar();
		return false;
	}

	int colorind = -1;
	int cubesind = -1;
	
	while( 1 ){

		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		//printf("%s\n",lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

		// else : parse lineHeader
		
		if ( strcmp( lineHeader, "v" ) == 0 ){
			glm::vec3 vertex;
			fscanf(file, "%f %f %f\n", &vertex.x, &vertex.z, &vertex.y );
			temp_vertices.push_back(vertex);
		}else if ( strcmp( lineHeader, "vt" ) == 0 ){
			glm::vec3 uv;
			fscanf(file, "%f %f\n", &uv.x, &uv.y );
			//uv.y = -uv.y; // Invert V coordinate since we will only use DDS texture, which are inverted. Remove if you want to use TGA or BMP loaders.
			//temp_uvs.push_back(colorbar[colorind]);
		}else if ( strcmp( lineHeader, "vn" ) == 0 ){
			glm::vec3 normal;
			fscanf(file, "%f %f %f\n", &normal.x, &normal.z, &normal.y );
			temp_normals.push_back(normal);
		}else if ( strcmp( lineHeader, "f" ) == 0 )
		{
			std::string vertex1, vertex2, vertex3;
			unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
			int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
			if (matches != 9)
			{
				printf("File can't be read by our simple parser :-( Try exporting with other options\n");
				fclose(file);
				return false;
			}

			vertexIndices.push_back(vertexIndex[0]);
			vertexIndices.push_back(vertexIndex[1]);
			vertexIndices.push_back(vertexIndex[2]);
			uvIndices    .push_back(uvIndex[0]);
			uvIndices    .push_back(uvIndex[1]);
			uvIndices    .push_back(uvIndex[2]);
			normalIndices.push_back(normalIndex[0]);
			normalIndices.push_back(normalIndex[1]);
			normalIndices.push_back(normalIndex[2]);

			if(cubesind != -1)
			{
				cubes[cubesind].push_back(triunit(vertexIndex[0] - 1,vertexIndex[1] - 1,vertexIndex[2] - 1,normalIndex[0] - 1));
			}

		}else{

			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
			//printf("%s\n",stupidBuffer);
			if ( strcmp( lineHeader, "o" ) == 0 )   //next mesh, change color 
			{
				colorind++;
				//printf("mesh name:%s\n",stupidBuffer);

				if ( strcmp( stupidBuffer, " arm1_Cube.002\n" ) == 0 ) { cubesind = 0; }// printf("record index:%d\n",cubesind);}
				else if ( strcmp( stupidBuffer, " arm2_Cube.003\n" ) == 0 )  { cubesind = 1;}// printf("record index:%d\n",cubesind);}
				else if ( strcmp( stupidBuffer, " arm3_Cube.004\n" ) == 0 )  { cubesind = 2;}// printf("record index:%d\n",cubesind);}
				else if ( strcmp( stupidBuffer, " arm4_Cube.005\n" ) == 0 )  { cubesind = 3;}// printf("record index:%d\n",cubesind);}
				else if ( strcmp( stupidBuffer, " body_Cube.001\n" ) == 0 )  { cubesind = 4;}// printf("record index:%d\n",cubesind);}
				else cubesind = -1;

			}
		}

		
	}

	cubecontours.resize(5);
	//cubecoord cubescoor[5];
	for(int i = 0 ; i < 5; i ++)
		cubecontours[i] = cubecoord(cubes[i],temp_vertices,temp_normals);

	fclose(file);

	return true;
}


