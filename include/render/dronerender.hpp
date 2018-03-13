
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "camera.hpp"
#include "mesh.hpp"
#include <GLFW/glfw3.h>
#include "landscape.hpp"

class dronerender
{
public :
dronerender();
dronerender(char * verterxpath,char * fragmentpath,char *modelpath,unsigned int width, unsigned int height, bool offscreen = false);  // --> build shader, load model and create camera,   init glfw and glew
dronerender(char * mv,char * mf,char *modelpath, char *lv, char *lf, std::vector<std::string>,unsigned int width, unsigned int height,bool offscreen = false);  // --> build shader, load model and create camera,   init glfw and glew

~dronerender();    //stop glfw and glew
bool  dorender(double posx, double posy, double posz,
			   double roll, double pitch,double yaw,
			   double posx_t,double posy_t,double posz_t,
			   double roll_t, double pitch_t,double yaw_t,
			   unsigned char* outputimg);//<---- hunter position, hunter orientation target position, target orientation, 

void fortest();
private:
	bool _offscreenflag;

	void init_offscreen();
	void deinit_offscreen();

	unsigned int SCR_WIDTH;
	unsigned int SCR_HEIGHT;
	
	dronemesh _dronemodel;
	dronelanscape _land;
	dronecamera _cam;
	GLFWwindow* _window;


	GLuint fbo;
	GLuint rbo_color;
	GLuint rbo_depth;
};
