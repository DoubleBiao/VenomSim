#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "camera.hpp"
#include "shader.h"
#include "mesh.hpp"
#include "landscape.hpp"

#include <GLFW/glfw3.h>
#include <vector>
#include <string>

class dronerender
{
public :
dronerender();

dronerender(char * mv,char * mf,char *modelpath, char *lv, char *lf, std::vector<std::string>);  // --> build shader, load model and create camera,   init glfw and glew

~dronerender();    //stop glfw and glew
bool  dorender(double posx, double posy, double posz,
			   double roll, double pitch,double yaw,
			   double posx_t,double posy_t,double posz_t,
			   double roll_t, double pitch_t,double yaw_t);//<---- hunter position, hunter orientation target position, target orientation, 

private:
	dronelanscape _land;
	dronemesh _dronemodel;
	dronecamera _cam;
	GLFWwindow* _window;


	//GLuint fbo;
	//GLuint rbo_color;
	//GLuint rbo_depth;
};
