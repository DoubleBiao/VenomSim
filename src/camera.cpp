#include "camera.hpp"
#include <cmath>
#include <iostream>


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

glm::mat3 getrotation3_inverse(glm::vec3 const & ori)
{
	using namespace std;
	float A = -glm::radians(ori[2]);
	float B = -glm::radians(ori[1]);
	float C = -glm::radians(ori[0]);

	//return glm::mat3(cos(A)*cos(B), cos(A)*sin(B)*sin(C)-sin(A)*cos(C), cos(A)*sin(B)*cos(C) + sin(A)*sin(C),
	//	        sin(A)*cos(B), sin(A)*sin(B)*sin(C)+cos(A)*cos(C),        sin(A)*sin(B)*cos(C) - cos(A)*sin(C) ,
	//			-sin(B),       cos(B)*sin(C),                      cos(B)*cos(C)                                ) ;

	return glm::mat3(cos(A)*cos(B),                        sin(A)*cos(B),                        -sin(B),
		             cos(A)*sin(B)*sin(C)-sin(A)*cos(C),   sin(A)*sin(B)*sin(C)+cos(A)*cos(C),   cos(B)*sin(C),
				     cos(A)*sin(B)*cos(C) + sin(A)*sin(C), sin(A)*sin(B)*cos(C) - cos(A)*sin(C), cos(B)*cos(C)) ;

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


dronecamera::dronecamera()
	:target(-1,0,0),up(0,0,1),right(0,1,0),world2drone(1.0f),projection(1.0f),viewport(0,0,0,0)
{}

dronecamera::dronecamera(glm::vec3  const & installori)
:target(-1,0,0),up(0,0,1),right(0,1,0),world2drone(1.0f),projection(1.0f),viewport(0,0,0,0)
{
	glm::mat3 R_install = getrotation3(installori);
	target = R_install*target;
	up = R_install*up;
	right = R_install*right;
}

dronecamera::dronecamera(float installroll, float installpitch, float installyaw)
:target(-1,0,0),up(0,0,1),right(0,1,0),world2drone(1.0f),projection(1.0f),viewport(0,0,0,0)
{
	glm::vec3 installori(installroll,installpitch,installyaw);

	glm::mat3 R_install = getrotation3(installori);
	target = R_install*target;
	up = R_install*up;
	right = R_install*right;
}

bool dronecamera::project(glm::vec3 const & origine,glm::vec3 & out2 )
{
	out2 = this->project_clipper(origine,world2drone,viewport);


	if(glm::isinf(out2[0]) || glm::isinf(out2[1]) || glm::isinf(out2[2])) return false;
	if(glm::isnan(out2[0]) || glm::isnan(out2[1]) || glm::isnan(out2[2]))
	{
		std::cout<<"in projection function: camare inside the drone"<<std::endl;
		return false;
	}
	return true;
}


glm::mat4 dronecamera::getfrustum()
{
    return projection;
};

glm::mat4 dronecamera::getviewpoint(glm::vec3 const &  cam_pos, glm::vec3  const & cam_ori)
{
	glm::mat3 R_body = getrotation3(cam_ori);
	glm::vec3 target_now = R_body * target;
	target_now = target_now + cam_pos;
	return glm::lookAt(cam_pos,target_now,R_body*up);
};

float dronecamera::getscreenarea()
{
	return SCR_WIDTH*SCR_HEIGHT;
}

void dronecamera::setfrustum(float left, float right, float bottom, float top, float nearVal, float farVal)
{
    projection = glm::frustum(left, right, bottom, top, nearVal, farVal);
};

glm::vec3 dronecamera::project_clipper(glm::vec3 const & obj, glm::mat4 const & model,glm::vec4 const & viewport)
{
	using namespace glm;

	vec4 tmp = vec4(obj, 1.0f);
	tmp = model * tmp;
	//clip:
	if(tmp[2] > 0) tmp[2] = -1e-3;

	tmp = projection * tmp;

	tmp /= tmp.w;

	tmp = tmp * 0.5f + 0.5f;
	tmp[0] = tmp[0] * viewport[2] + viewport[0];
	tmp[1] = tmp[1] * viewport[3] + viewport[1];

	return vec3(tmp);

}

void dronecamera::installcamera(float installroll, float installpitch, float installyaw)
{
	glm::vec3 installori(installroll,installpitch,installyaw);
	glm::vec3 defaultarget(-1,0,0);
	glm::vec3 defaulupup(0,0,1);
	glm::vec3 defaultright(0,1,0);

	glm::mat3 R_install = getrotation3(installori);
	target = R_install*defaultarget;
	up = R_install*defaulupup;
	right = R_install*defaultright;
}


void printmat4(glm::mat4 mat,char * name)
{
	using namespace std;
	cout<<endl;
	cout<<name<<endl;
	cout<<mat[0][0]<<","<<mat[0][1]<<","<<mat[0][2]<<","<<mat[0][3]<<","<<endl;
	cout<<mat[1][0]<<","<<mat[1][1]<<","<<mat[1][2]<<","<<mat[1][3]<<","<<endl;
	cout<<mat[2][0]<<","<<mat[2][1]<<","<<mat[2][2]<<","<<mat[2][3]<<","<<endl;
	cout<<mat[3][0]<<","<<mat[3][1]<<","<<mat[3][2]<<","<<mat[3][3]<<","<<endl;


}
void dronecamera::setdroneposture(float dronex, float droney, float dronez,float droneroll, float dronepitch, float droneyaw,
								  float tx, float ty, float tz, float troll, float tpitch, float tyaw)
{
	glm::mat4 model(1.0f);


	glm::vec3 targetposition(tx,ty,tz);
	glm::vec3 targetori(troll,tpitch,tyaw);

    model = glm::translate(glm::mat4(1.0f),targetposition);
    model = model*getrotation4(targetori); 

	
	////////////to do : translate into target coordinate///////////////

	//in: drone postion, drone orientation
	//out: drone postion, drone orientation

	///////////////////////
	world2drone = this->getviewpoint(glm::vec3(dronex,droney,dronez),glm::vec3(droneroll,dronepitch,droneyaw))*model;
}


void dronecamera::setscreensize(float width, float height)
{
	SCR_WIDTH = width;
	SCR_HEIGHT = height;
	viewport[0] = 0;viewport[1] = 0;
	viewport[2] = SCR_WIDTH;
	viewport[3] = SCR_HEIGHT;

	viewrect.clear();
	viewrect.push_back(glm::vec2(0,SCR_HEIGHT));viewrect.push_back(glm::vec2(SCR_WIDTH,SCR_HEIGHT));
	viewrect.push_back(glm::vec2(SCR_WIDTH,0));viewrect.push_back(glm::vec2(0,0));
}
