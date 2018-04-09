#include "virtulvision.hpp"
#include "polygonetool.hpp"
#include <iostream>
#include <vector>

//dronecamera cam(0.0f,0.0f,0.0f);
//imagecoor * uwcoor;

int clipind[10] = {0,0,0,1,1,2,0,1,2,3};
int subjind[10] = {1,2,3,2,3,3,4,4,4,4};


glm::vec3 getcamposindronecoor(glm::vec3 const & campos,glm::vec3 const &  targetpos, glm::vec3 const &  targetori)
{
	glm::vec3 tmp = campos - targetpos;
	glm::mat3 R =  getrotation3_inverse(targetori);
	
	return std::move(R*tmp);

}

void virtulvision::loaddrone(const char * modelfilename)
{
	dronecontour.clear();
	loadcubes(modelfilename,dronecontour);
}


void virtulvision::installcamera(float roll, float pitch, float yaw,
                       float FOVleft, float FOVright, float FOVbottom, float FOVtop, float FOVnear, float FOVfar,
					   float screenwidth, float screenheight)
{
	ondronecam.installcamera(roll,pitch,yaw);
	ondronecam.setfrustum(FOVleft,FOVright,FOVbottom,FOVtop,FOVnear,FOVfar);
	ondronecam.setscreensize(screenwidth, screenheight);
}

bool virtulvision::projectcontor(std::vector<glm::vec3> contour,std::vector<glm::vec2> & projectedcoor)
{
	projectedcoor.clear();

	for(auto vertex:contour)
	{
		glm::vec3 tmp;
		if(this->ondronecam.project(vertex,tmp) == false)
		{// if any vertex is projected to be nan or inf, the drone is regarded definitly dispear from the view cone. 
			projectedcoor.clear();
			return false;
		}
		projectedcoor.push_back(glm::vec2(tmp[0],tmp[1]));
	}

	return true;
}

void virtulvision::seedrone(float hx,float hy, float hz, float hroll, float hpitch, float hyaw,
							float tx, float ty, float tz, float troll, float tpitch, float tyaw,
							imagecoor * outputinfo)
{
	this->ondronecam.setdroneposture(hx,hy,hz,hroll,hpitch,hyaw,tx,ty,tz,troll,tpitch,tyaw);
	glm::vec3 campos(hx,hy,hz);
	glm::vec3 camposintarget = getcamposindronecoor(campos,glm::vec3(tx,ty,tz),glm::vec3(troll,tpitch,tyaw));

	std::vector<std::vector<glm::vec2>> contours2d;
	std::vector<std::vector<glm::vec2>> clippedcontours2d;

	bool projectionflag = true;
	float insightarea = 0;

	

	for(auto contour:dronecontour)
	{
		std::vector<glm::vec2> projcontour;
		projectionflag = this->projectcontor(contour.contourpoint(camposintarget),projcontour);
		if(!projectionflag) break;   //if any vertex of the drone is projected to be nan or inf, something wrong must happen. Then area = 0
		contours2d.push_back(projcontour);
	}

	if(projectionflag)
	{
		for(auto rawcontour:contours2d)
		{
			clippedcontours2d.push_back(polyclip(ondronecam.viewrect,rawcontour));
			insightarea += polyarea(clippedcontours2d.back());
		}

		std::vector<std::vector<glm::vec2>> overlappart;
		if(fabs(insightarea) > 1e-10)
		{
			//clippedcontour: 0:arm1, 1:arm2, 2: arm3, 3: arm4, 4: body
			//area to compute S(arm1), S(arm2), S(arm3), S(arm4), S(body)
			//subtract: S(arm1 & arm2), S(arm1 & arm3), S(arm1 & arm4), S(arm2 & arm3), S(arm2 & arm4), S(arm3 & arm4)
			//subrract: S(arm1 & body), S(arm2 & body), S(arm3 & body), S(arm4 & body)
			//total --> 10
			

			for(int i = 0; i < 10; i++)
			{
				if(clippedcontours2d[clipind[i]].empty() || clippedcontours2d[subjind[i]].empty()) continue;
				overlappart.push_back(polyclip(clippedcontours2d[clipind[i]],clippedcontours2d[subjind[i]]));
				insightarea -= polyarea(overlappart.back());
			}
		}
		//drawineps("test6.eps",contours2d,ondronecam.viewrect,clippedcontours2d);
	}



	insightarea /= ondronecam.getscreenarea();

	glm::vec3 projectedcenter;
	this->ondronecam.project(glm::vec3(0,0,0),projectedcenter);

	outputinfo->u = projectedcenter[0];
	outputinfo->v = projectedcenter[1];
	outputinfo->w = projectedcenter[2];



	outputinfo->area = insightarea;


	
}
