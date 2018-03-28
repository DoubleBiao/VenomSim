#include "lag.hpp"
#include <iostream>

lagmodule::lagmodule(){}

lagmodule::~lagmodule(){}

lagmodule::lagmodule(int sim_span, int delayspan)  
{	
}

bool lagmodule::behavetriggerd(int digitaltime)
{
	return digitaltime == delay_shift;
}


senselag::senselag(){}

senselag::~senselag(){}

senselag::senselag(int sim_span, int delayspan)
{
	delay_shift = (sim_span - delayspan % sim_span)%sim_span;
	delay_order = (int)delayspan/sim_span + 1;
	//lagmodule(sim_span,delayspan);

	std::cout<<"senselag: "<<"delay shift: "<<delay_shift<<std::endl;
	std::cout<<"delay order: "<<delay_order<<std::endl;
}

bool senselag::bufferfull()
{
	//std::cout<<"bufferfull:"<<"buffersize:"<<infobuffer.size()<<" delay_order: "<<delay_order<<std::endl;
	return infobuffer.size()== delay_order;
}
//time to send out data to simulator user, it will return the last element out of queue
void senselag::outputinfo(infoformat* info)
{
	//if the queue is not full, it will return zero data
	//std::cout<<"outputinfo:"<<"buffersize:"<<infobuffer.size()<<" delay_order: "<<delay_order<<std::endl;
	if(infobuffer.size()!= delay_order)
	{
		info->posx = 0;info->posy = 0;info->posz = 0;
		info->velocityx = 0;info->velocityy = 0;info->velocityz = 0;
		info->thetax = 0;info->thetay = 0;info->thetaz = 0;
		info->accx = 0;info->accy = 0;info->accz = 0;

		info->posx_t = 0;info->posy_t = 0;info->posz_t = 0;
		info->velocityx_t = 0;info->velocityy_t = 0;info->velocityz_t = 0;
		info->thetax_t = 0;info->thetay_t = 0;info->thetaz_t = 0;
		info->accx_t = 0;info->accy_t = 0;info->accz_t = 0;

		info->thrust = 0;
	}
	else
	{
		infoformat latestinfo = infobuffer.front();
		infobuffer.pop();
		
		info->posx = latestinfo.posx;info->posy = latestinfo.posy;info->posz = latestinfo.posz;
		info->velocityx = latestinfo.velocityx;info->velocityy = latestinfo.velocityy;info->velocityz = latestinfo.velocityz;
		info->thetax = latestinfo.thetax;info->thetay = latestinfo.thetay;info->thetaz = latestinfo.thetaz;
		info->accx = latestinfo.accx;info->accy = latestinfo.accy;info->accz = latestinfo.accz;
		
		info->posx_t = latestinfo.posx_t;info->posy_t = latestinfo.posy_t;info->posz_t = latestinfo.posz_t;
		info->velocityx_t = latestinfo.velocityx_t;info->velocityy_t = latestinfo.velocityy_t;info->velocityz_t = latestinfo.velocityz_t;
		info->thetax_t = latestinfo.thetax_t;info->thetay_t = latestinfo.thetay_t;info->thetaz_t = latestinfo.thetaz_t;
		info->accx_t = latestinfo.accx_t;info->accy_t = latestinfo.accy_t;info->accz_t = latestinfo.accz_t;

		info->thrust = latestinfo.thrust;
	}
}

//sense the data from the sensor
void senselag::readinfo(quadcopter * hunter, quadcopter * target, bool initflag)
{
	infoformat newinfo;
	newinfo.posx = hunter->get_position(0);
	newinfo.posy = hunter->get_position(1);
	newinfo.posz = hunter->get_position(2);

	newinfo.velocityx = hunter->get_speed(0);
	newinfo.velocityy = hunter->get_speed(1);
	newinfo.velocityz = hunter->get_speed(2);

	newinfo.thetax = hunter->get_attitude(0);
	newinfo.thetay = hunter->get_attitude(1);
	newinfo.thetaz = hunter->get_attitude(2);

	newinfo.accx = hunter->get_acc(0);
	newinfo.accy = hunter->get_acc(1);
	newinfo.accz = hunter->get_acc(2);

	newinfo.posx_t = target->get_position(0);
	newinfo.posy_t = target->get_position(1);
	newinfo.posz_t = target->get_position(2);

	newinfo.velocityx_t = target->get_speed(0);
	newinfo.velocityy_t = target->get_speed(1);
	newinfo.velocityz_t = target->get_speed(2);

	newinfo.thetax_t = target->get_attitude(0);
	newinfo.thetay_t = target->get_attitude(1);
	newinfo.thetaz_t = target->get_attitude(2);

	newinfo.accx_t = target->get_acc(0);
	newinfo.accy_t = target->get_acc(1);
	newinfo.accz_t = target->get_acc(2);

	newinfo.thrust = hunter->get_thrust();

	if(initflag == true) newinfo.thrust = 0;

	infobuffer.push(newinfo);

	//std::cout<<"readinfo"<<"buffersize"<<infobuffer.size()<<std::endl;
}


controllag::controllag(){}

controllag::~controllag(){}

controllag::controllag(int sim_span, int delayspan)
{
	delay_shift = delayspan % sim_span;
	delay_order = (int)delayspan/sim_span + 1;
	//lagmodule(sim_span,delayspan);

	std::cout<<"controllag: "<<"delay shift: "<<delay_shift<<std::endl;
	std::cout<<"delay order: "<<delay_order<<std::endl;
}

//time to read the control commend from the simulator usr
 void controllag::readcmd(double roll, double pitch, double yaw, double throttle,
					 double roll_t, double pitch_t, double yaw_t, double throttle_t)
 {
	 cmdformat newcmd = {roll,pitch,yaw,throttle,roll_t,pitch_t,yaw_t,throttle_t};
	 cmdbuffer.push(newcmd);

	 //std::cout<<"readcmd"<<"buffersize"<<cmdbuffer.size()<<std::endl;
 }

 //carry out the control commend after the lag
 cmdformat controllag::carryoutcmd()
 {
	 //if cmdbuffer is not full, just return zero cmd
	 if(cmdbuffer.size()!= delay_order)
	 {
		 cmdformat newcmd = {0,0,0,0,    0,0,0,0};
		 return std::move(newcmd);
	 }
	 else
	 {
		 cmdformat newcmd = cmdbuffer.front();
		 cmdbuffer.pop();
		 return std::move(newcmd);
	 }

 }
