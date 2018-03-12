#ifndef CAMERA
#define CAMERA

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <cmath>


glm::mat4 getrotation4(glm::vec3 const & ori);
struct imagecoor
{
    double u;
    double v;
	double w;
};

class dronecamera
{
private:
	glm::vec3 target;
	glm::vec3 up;
	glm::vec3 right;

public:
	dronecamera();
        dronecamera(double row, double pitch, double yaw);
	dronecamera(glm::vec3  const & installori);
	glm::mat4 getviewpoint(glm::vec3  const & cam_pos, glm::vec3  const & cam_ori);
	glm::mat4 getlandview(glm::vec3  const & cam_pos, glm::vec3  const & cam_ori);
};
#endif
