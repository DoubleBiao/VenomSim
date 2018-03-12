#ifndef MESH
#define MESH

#include"shader.h"
//#include <GL/glew.h>
#include <glad/glad.h>
#include <vector>
#include "objloader.hpp"



class dronemesh
{
private:
	Shader _shader;
	GLuint _VBO; 
	GLuint _VAO;
	GLuint _MatrixID;
	std::vector<float> _vertexdata;
	int _vertexnum;

public:
	dronemesh(){};
	dronemesh(char * vertexshader, char * fragmentshader, char * modelpath):_shader(vertexshader,fragmentshader) 
	{
		glm::vec3 colorbar[] = 
		{
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller1
			glm::vec3(1.0f, 0.0f, 0.0f),     //arm1
			glm::vec3(0.657f, 0.630f, 1.0f), //body
			glm::vec3(0.024f, 0.936f,1.0f),  //arm2
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller2
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller3
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller4
			glm::vec3(0.0f, 0.0f, 1.0f),     //arm3
			glm::vec3(1.0f, 0.657f, 0.023f)    //arm4
		};
		bool res = 	loadOBJwithcolor(modelpath,_vertexdata,& _vertexnum,colorbar);

		_shader.Use();

		glGenVertexArrays(1, &_VAO);
		glGenBuffers(1, &_VBO);

		glBindVertexArray(_VAO);

		glBindBuffer(GL_ARRAY_BUFFER, _VBO);
		glBufferData(GL_ARRAY_BUFFER, _vertexdata.size()*sizeof(float), _vertexdata.data(), GL_STATIC_DRAW);

		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		// color attribute
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		_MatrixID = glGetUniformLocation(_shader.Program, "MVP");
	};
	void loadprg(char * vertexshader, char * fragmentshader, char * modelpath)
	{
		_shader.loadshader(vertexshader,fragmentshader);

		glm::vec3 colorbar[] = 
		{
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller1
			glm::vec3(1.0f, 0.0f, 0.0f),     //arm1
			glm::vec3(0.657f, 0.630f, 1.0f), //body
			glm::vec3(0.024f, 0.936f,1.0f),  //arm2
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller2
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller3
			glm::vec3(0.0f, 0.0f, 0.0f),     //propeller4
			glm::vec3(0.0f, 0.0f, 1.0f),     //arm3
			glm::vec3(1.0f, 0.657f, 0.023f)    //arm4
		};
		bool res = 	loadOBJwithcolor(modelpath,_vertexdata,& _vertexnum,colorbar);

		_shader.Use();

		glGenVertexArrays(1, &_VAO);
		glGenBuffers(1, &_VBO);

		glBindVertexArray(_VAO);

		glBindBuffer(GL_ARRAY_BUFFER, _VBO);
		glBufferData(GL_ARRAY_BUFFER, _vertexdata.size()*sizeof(float), _vertexdata.data(), GL_STATIC_DRAW);

		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		// color attribute
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		_MatrixID = glGetUniformLocation(_shader.Program, "MVP");
	};
	~dronemesh()
	{
		glDeleteVertexArrays(1, &_VAO);
		glDeleteBuffers(1, &_VBO);
	};

	void draw(glm::mat4 & MVP)
	{
		_shader.Use();
		glUniformMatrix4fv(_MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glBindVertexArray(_VAO);
        //glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		glDrawArrays(GL_TRIANGLES, 0, _vertexnum);
	}


};
#endif