//#include <GL/glew.h>
#include "dronerender.hpp"


const unsigned int SCR_WIDTH = 400;
const unsigned int SCR_HEIGHT = 300;

void processInput(GLFWwindow *window);

dronerender::dronerender(char * mv,char * mf,char *modelpath, char *lv, char *lf, std::vector<std::string> faces)
	:_cam(glm::vec3(0,30,180))
{
		//init glfw and glew
	glfwInit();
 //   // Set all the required options for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	//glfwWindowHint(GLFW_VISIBLE,GL_FALSE);
    // Create a GLFWwindow object that we can use for GLFW's functions
	_window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", nullptr, nullptr);
	glfwMakeContextCurrent(_window);


    //glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED );//GLFW_CURSOR_DISABLED);
    // Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
    //glewExperimental = GL_TRUE;

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return ;
	}    
	
	_dronemodel.loadprg(mv,mf,modelpath);
	_land.loadprg(lv,lf,faces);

	
	glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);

	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	//// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

}

dronerender::dronerender(char * verterxpath,char * fragmentpath,char *modelpath)
	:_cam(glm::vec3(0,30,180))
{
	//init glfw and glew
	glfwInit();
 //   // Set all the required options for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	//glfwWindowHint(GLFW_VISIBLE,GL_FALSE);
    // Create a GLFWwindow object that we can use for GLFW's functions
	_window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", nullptr, nullptr);
	glfwMakeContextCurrent(_window);


    glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED );//GLFW_CURSOR_DISABLED);
    // Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
    //glewExperimental = GL_TRUE;
	//glewInit();
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return ;
	}    
	//_dronemodel = dronemesh(verterxpath,fragmentpath,modelpath);
	_dronemodel.loadprg(verterxpath,fragmentpath,modelpath);
	glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	//// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);
}

dronerender::~dronerender()
{
	glfwTerminate();
}

bool dronerender::dorender(double posx, double posy, double posz,
						   double roll, double pitch,double yaw,
						   double posx_t,double posy_t,double posz_t,
						   double roll_t, double pitch_t,double yaw_t)
{
	//---------------------------------------compute mvp matrix-------------------------------------------//
	glm::vec3 targetposition(posx_t,posy_t,posz_t);
	glm::vec3 targetori(roll_t,pitch_t,yaw_t);
	
	glm::vec3 hunterpos(posx,posy,posz);
	glm::vec3 hunterori(roll,pitch,yaw);

	//glm::vec3 targetposition(0.0f,0,0.0f);
	//glm::vec3 targetori(0.0f,0.0f,45.0f);
	//
	//glm::vec3 hunterpos(10.0f,0,10.0f);
	//glm::vec3 hunterori(0,-45.0f,0);


	//glm::mat4 model(1.0f);
	glm::mat4 model = glm::translate(glm::mat4(1.0f),targetposition);
	model = model*getrotation4(targetori); 

	glm::mat4 view = _cam.getviewpoint(hunterpos,hunterori);
	auto projection = glm::perspective(glm::radians(45.0f),  (float)SCR_WIDTH/(float)SCR_HEIGHT, 2.0f, 500.0f);

	glm::mat4 MVP =  projection * view * model;//glm::rotate(glm::mat4(), glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f));


    //-----------------------------------------render---------------------------------------------//
	if (!glfwWindowShouldClose(_window))
	{
		processInput(_window);
		//processInput(window);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		_dronemodel.draw(MVP);
                auto lview = _cam.getlandview(hunterpos,hunterori);
		_land.draw(lview,projection);

		glfwSwapBuffers(_window);
		glfwPollEvents();

		return true;
	}
	return false;
}




// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}
// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

dronerender::dronerender()
{}

