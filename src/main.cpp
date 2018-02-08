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



quadcopter * Quadcopter_sim;
quadcopter * Quadcopter_target;
infoformat * outputbuffer;

//init interface 
extern "C" void siminit(double initx, double inity, double initz, double initroll, double initpitch, double inityaw,
											 double initx_t, double inity_t, double initz_t, double initroll_t, double initpitch_t, double inityaw_t)
{
	//agent 
	Quadcopter_sim = new quadcopter;
	Quadcopter_sim->startSimulation(initx,inity,initz,initroll,initpitch,inityaw);

	//target
	Quadcopter_target = new quadcopter;
	Quadcopter_target->startSimulation(initx_t,inity_t,initz_t,initroll_t,initpitch_t,inityaw_t);

	outputbuffer = new infoformat;
}


//getcommand interface
extern "C" void simcontrol(double roll,double pitch, double yaw, double throttle,
			   double roll_t, double pitch_t, double yaw_t, double throttle_t)
{
	Quadcopter_sim->getcommands(roll,pitch,yaw,throttle);

	Quadcopter_target->getcommands(roll_t,pitch_t,yaw_t,throttle_t);
}


//run interface 
extern "C" void simrun(unsigned long long period)
{
	//std::cout<<period<<std::endl;
	Quadcopter_sim->dosimulating(period);
	Quadcopter_target->dosimulating(period);
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


//release interface
extern "C" void simstop()
{
	delete Quadcopter_sim;
	delete Quadcopter_target;
	delete outputbuffer;
}
