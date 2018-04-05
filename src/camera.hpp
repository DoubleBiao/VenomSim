#ifndef CAMERA
#define CAMERA

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

glm::mat4 getrotation4(glm::vec3 const & ori);

glm::mat3 getrotation3_inverse(glm::vec3 const & ori);

class dronecamera
{
private:
	glm::vec3 target;
	glm::vec3 up;
	glm::vec3 right;
	glm::mat4 projection;
	glm::vec4 viewport;
	glm::mat4 world2drone;
	

	float SCR_WIDTH;
	float SCR_HEIGHT;

	

public:
	std::vector<glm::vec2> viewrect;


	dronecamera();
	dronecamera(glm::vec3  const & installori);
	dronecamera(float installroll, float installpitch, float installyaw);
	glm::mat4 getviewpoint(glm::vec3  const & cam_pos, glm::vec3  const & cam_ori);
	
	glm::mat4 getfrustum();

	glm::vec3 project_clipper(glm::vec3 const & obj, glm::mat4 const & model, glm::vec4 const & viewport);

	void setfrustum(float left, float right, float bottom, float top, float nearVal, float farVal);
	void installcamera(float installroll, float installpitch, float installyaw);
	void setdroneposture(float x, float y, float z,float roll, float pitch, float yaw,
                         float tx, float ty, float tz, float troll, float tpitch, float tyaw);
	void setscreensize(float width, float height);
	
	float getscreenarea();
	bool project(glm::vec3 const & origine,glm::vec3 & out2);

};
#endif