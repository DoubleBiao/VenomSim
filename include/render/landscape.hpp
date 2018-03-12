#include "shader.h"
#include <string>
#include <vector>

class dronelanscape
{
private:
	Shader _shader;
	GLuint _VBO; 
	GLuint _VAO;
	GLuint _MatrixID;
	unsigned int cubemapTexture;
public:
	dronelanscape();
	void loadprg(char * vertexshader, char * fragmentshader,std::vector<std::string> faces);
	void draw(glm::mat4 & initview, glm::mat4 & projection);
	~dronelanscape();
};

