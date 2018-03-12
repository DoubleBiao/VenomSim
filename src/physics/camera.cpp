#include "camera.hpp"


glm::mat4 getrotation4(glm::vec3 const & ori)
{
	using namespace std;
	float A = -glm::radians(ori[2]);
	float B = -glm::radians(ori[1]);
	float C = -glm::radians(ori[0]);

	return glm::mat4(cos(A)*cos(B), cos(A)*sin(B)*sin(C)-sin(A)*cos(C), cos(A)*sin(B)*cos(C) + sin(A)*sin(C),0.0f,
		        sin(A)*cos(B), sin(A)*sin(B)*sin(C)+cos(A)*cos(C), sin(A)*sin(B)*cos(C) - cos(A)*sin(C),0.0f,
				-sin(B),       cos(B)*sin(C),                      cos(B)*cos(C)                       ,0.0f,
				0.0f,          0.0f,                               0.0f,                                1.0f);  
	//return glm::mat4 R;
}

glm::mat3 getrotation3(glm::vec3 const & ori)
{
	using namespace std;
	float A = -glm::radians(ori[2]);
	float B = -glm::radians(ori[1]);
	float C = -glm::radians(ori[0]);

	return glm::mat3(cos(A)*cos(B), cos(A)*sin(B)*sin(C)-sin(A)*cos(C), cos(A)*sin(B)*cos(C) + sin(A)*sin(C),
		        sin(A)*cos(B), sin(A)*sin(B)*sin(C)+cos(A)*cos(C), sin(A)*sin(B)*cos(C) - cos(A)*sin(C),
				-sin(B),       cos(B)*sin(C),                      cos(B)*cos(C)                           );  
}


void printmatrix(char *name, glm::mat4 const & mat)
{
	using namespace std;
	cout<<name<<":"<<endl;
	cout<<mat[0][0]<<","<<mat[0][1]<<","<<mat[0][2]<<","<<mat[0][3]<<endl;
	cout<<mat[1][0]<<","<<mat[1][1]<<","<<mat[1][2]<<","<<mat[1][3]<<endl;
	cout<<mat[2][0]<<","<<mat[2][1]<<","<<mat[2][2]<<","<<mat[2][3]<<endl;
	cout<<mat[3][0]<<","<<mat[3][1]<<","<<mat[3][2]<<","<<mat[3][3]<<endl;
	cout<<endl;
}


dronecamera::dronecamera():target(-1,0,0),up(0,0,1),right(0,1,0){}

dronecamera::dronecamera(double row, double pitch, double yaw):target(-1,0,0),up(0,0,1),right(0,1,0)
{
        glm::vec3 installori(row,pitch,yaw);
	glm::mat3 R_install = getrotation3(installori);
	target = R_install*target;
	up = R_install*up;
	right = R_install*right;
}



dronecamera::dronecamera(glm::vec3  const & installori):target(-1,0,0),up(0,0,1),right(0,1,0)
{
	glm::mat3 R_install = getrotation3(installori);
	target = R_install*target;
	up = R_install*up;
	right = R_install*right;
}


glm::mat4 dronecamera::getviewpoint(glm::vec3 const &  cam_pos, glm::vec3  const & cam_ori)
{
	glm::mat3 R_body = getrotation3(cam_ori);
	glm::vec3 target_now = R_body * target;
	target_now = target_now + cam_pos;
	return glm::lookAt(cam_pos,target_now,R_body*up);
};

glm::mat4 dronecamera::getlandview(glm::vec3 const &  cam_pos, glm::vec3  const & cam_ori)
{
	glm::mat3 R_land = getrotation3(glm::vec3(-90,0,90));
	glm::mat3 R_body = R_land * getrotation3(cam_ori);
	glm::vec3 target_now = R_body* target;
	glm::vec3 upnow = R_body*up;
	target_now = target_now + cam_pos;
	
	return glm::lookAt(cam_pos,target_now,upnow);
}
