/*

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#include <thread>
#include <time.h>
#include <chrono>

#include "main_config.h"
#include "quadcopter.h"

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "virtulvision.hpp"

const unsigned long long samplingperiod = 1e9/360;

quadcopter * Quadcopter_sim;
quadcopter * Quadcopter_target;
infoformat * outputbuffer;


//init interface 
extern "C" void siminit(double initx, double inity, double initz, double initroll, double initpitch, double inityaw,
											 double initx_t, double inity_t, double initz_t, double initroll_t, double initpitch_t, double inityaw_t, double speed_upbound_hunter,double speed_upbound_target,
                        double yawdot_bound, double yawdot_bound_t)
{
	if(Quadcopter_sim != nullptr)  delete Quadcopter_sim;
	if(Quadcopter_target != nullptr) delete Quadcopter_target;
	if(outputbuffer == nullptr)  outputbuffer = new infoformat;      

	Quadcopter_sim = new quadcopter;
	Quadcopter_sim->startSimulation(initx,inity,initz,initroll,initpitch,inityaw,speed_upbound_hunter,yawdot_bound);

	Quadcopter_target = new quadcopter;
	Quadcopter_target->startSimulation(initx_t,inity_t,initz_t,initroll_t,initpitch_t,inityaw_t,speed_upbound_target,yawdot_bound_t);
}


//run interface 
extern "C" void simrun(double roll, double pitch, double yaw, double throttle,
		       double roll_t, double pitch_t, double yaw_t, double throttle_t,
                       unsigned long long period)
{
    unsigned long long simspan = period;
    while(simspan > samplingperiod)
    {
        Quadcopter_sim->getcommands(roll,pitch,yaw,throttle);
        Quadcopter_sim->dosimulating(QS_TIME_DELTA,samplingperiod);

        Quadcopter_target->getcommands(roll_t,pitch_t,yaw_t,throttle_t);
        Quadcopter_target->dosimulating(QS_TIME_DELTA,samplingperiod);


        simspan -= samplingperiod;
    }
    if(simspan > 0)
    {
        Quadcopter_sim->getcommands(roll,pitch,yaw,throttle);
        Quadcopter_sim->dosimulating(simspan,simspan);

        Quadcopter_target->getcommands(roll_t,pitch_t,yaw_t,throttle_t);
        Quadcopter_target->dosimulating(simspan,simspan);
    }
}

//fetchinfo interface
extern "C" infoformat * siminfo()
{
	outputbuffer->posx = Quadcopter_sim->get_position(0);
	outputbuffer->posy = Quadcopter_sim->get_position(1);
	outputbuffer->posz = Quadcopter_sim->get_position(2);

	outputbuffer->velocityx = Quadcopter_sim->get_speed(0);
	outputbuffer->velocityy = Quadcopter_sim->get_speed(1);
	outputbuffer->velocityz = Quadcopter_sim->get_speed(2);

	outputbuffer->thetax = Quadcopter_sim->get_attitude(0);
	outputbuffer->thetay = Quadcopter_sim->get_attitude(1);
	outputbuffer->thetaz = Quadcopter_sim->get_attitude(2);

	outputbuffer->accx = Quadcopter_sim->get_acc(0);
	outputbuffer->accy = Quadcopter_sim->get_acc(1);
	outputbuffer->accz = Quadcopter_sim->get_acc(2);

	outputbuffer->posx_t = Quadcopter_target->get_position(0);
	outputbuffer->posy_t = Quadcopter_target->get_position(1);
	outputbuffer->posz_t = Quadcopter_target->get_position(2);

	outputbuffer->velocityx_t = Quadcopter_target->get_speed(0);
	outputbuffer->velocityy_t = Quadcopter_target->get_speed(1);
	outputbuffer->velocityz_t = Quadcopter_target->get_speed(2);

	outputbuffer->thetax_t = Quadcopter_target->get_attitude(0);
	outputbuffer->thetay_t = Quadcopter_target->get_attitude(1);
	outputbuffer->thetaz_t = Quadcopter_target->get_attitude(2);

	outputbuffer->accx_t = Quadcopter_target->get_acc(0);
	outputbuffer->accy_t = Quadcopter_target->get_acc(1);
	outputbuffer->accz_t = Quadcopter_target->get_acc(2);

	outputbuffer->thrust = Quadcopter_sim->get_thrust();
	//std::cout<<outputbuffer->posx<<outputbuffer->posy<<outputbuffer->posz<<std::endl;

	return outputbuffer;
}







imagecoor * uwcoor;
virtulvision projector;

extern "C" void installcamera(double roll, double pitch, double yaw,
                                double FOVleft, double FOVright, double FOVbottom, double FOVtop, double FOVnear, double FOVfar,
				double screenwidth, double screenheight)
{
	projector.loaddrone("simpledrone.obj");
	projector.installcamera(roll,pitch,yaw,
			FOVleft,FOVright,FOVbottom,FOVtop,FOVnear,FOVfar,
			screenwidth,screenheight);

	if(uwcoor == nullptr) uwcoor = new imagecoor; 


	//std::cout<<"in installcamera:\n"<<std::endl;
	//std::cout<<"camori:"<<roll<<","<<pitch<<","<<yaw<<std::endl;
	//std::cout<<"frustum:"<<FOVleft<<","<<FOVright<<","<<FOVbottom<<","<<FOVtop<<","<<FOVnear<<","<<FOVfar<<std::endl;
	//std::cout<<"screen size"<<screenwidth<<","<<screenheight<<std::endl;

}

extern "C" imagecoor * simprojection(
                              double hx,double hy, double hz, double hroll, double hpitch,double hyaw,
                              double tx,double ty, double tz, double troll, double tpitch,double tyaw)
{

	//std::cout<<"in simprojection:\n"<<std::endl;
	//std::cout<<"hunter position: "<<hx<<","<<hy<<","<<hz<<std::endl;
	//std::cout<<"hunter orientation: "<<hroll<<","<<hpitch<<","<<hyaw<<std::endl;
	//std::cout<<"target position: "<<tx<<","<<ty<<","<<tz<<std::endl;
	//std::cout<<"target orientation: "<<troll<<","<<tpitch<<","<<tyaw<<std::endl;



	projector.seedrone(hx,hy,hz,hroll,hpitch,hyaw,
	tx,ty,tz,troll,tpitch,tyaw,
	uwcoor);

	//std::cout<<"out data:\n"<<std::endl;
	//std::cout<<uwcoor->u<<","<<uwcoor->v<<","<<uwcoor->w<<","<<uwcoor->area<<std::endl;

	return uwcoor;
}


//release interface
extern "C" void simstop()
{
	if(Quadcopter_sim!= nullptr)     delete Quadcopter_sim;
	if(Quadcopter_target != nullptr) delete Quadcopter_target;
	if(outputbuffer != nullptr )     delete outputbuffer;
        if(uwcoor!= nullptr)             delete uwcoor;


       	outputbuffer = nullptr;
	Quadcopter_sim = nullptr;
	Quadcopter_target = nullptr;
        uwcoor = nullptr;
}

