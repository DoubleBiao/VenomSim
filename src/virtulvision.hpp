#include <vector>
#include <glm/glm.hpp>
#include "visiondrone.hpp"
#include "camera.hpp"

using namespace std;

struct imagecoor
{
    double u;
    double v;
    double w;
    double area;
};



class virtulvision
{
private:
	cubecoord dronebody;
	std::vector<cubecoord> dronecontour;
	dronecamera ondronecam;

public:
	void installcamera(float roll, float pitch, float yaw,
                       float FOVleft, float FOVright, float FOVbottom, float FOVtop, float FOVnear, float FOVfar,
					   float screenwidth, float screenheight);
	
	bool projectcontor(std::vector<glm::vec3> contour,std::vector<glm::vec2> & projectedcoor);

	void loaddrone(const char * modelfilename);

	void seedrone(float hx, float hy, float hz, float hroll, float hpitch, float hyaw,
		          float tx, float ty, float tz, float troll, float tpitch, float tyaw,
		          imagecoor * outputinfo);
};

glm::vec3 getcamposindronecoor(glm::vec3 const & campos,glm::vec3 const &  targetpos, glm::vec3 const &  targetori);
