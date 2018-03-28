#include<queue>
#include"quadcopter.h"

class lagmodule
{
protected:
	int delay_shift;
	int delay_order;
	
    //std::queue<Info> infoqueue;

public:
	lagmodule();
	lagmodule(int sim_span, int delayspan);
	~lagmodule();

	bool behavetriggerd(int digitaltime);
};

class senselag:public lagmodule
{
private:
	std::queue<infoformat> infobuffer;

public:
	senselag();
	senselag(int sim_span, int delayspan);  //sim_span = _period/samplingperiod, delayspan = delay/samplingperiod
	~senselag();

	void outputinfo(infoformat* info);
	void readinfo(quadcopter * hunter, quadcopter * target, bool initflag);
	bool bufferfull();
};

class controllag:public lagmodule
{
private:
	std::queue<cmdformat> cmdbuffer;

public:
	controllag();
	controllag(int sim_span, int delayspan);
	~controllag();

	void readcmd(double roll, double pitch, double yaw, double throttle,
					 double roll_t, double pitch_t, double yaw_t, double throttle_t);
	
	cmdformat carryoutcmd();
};