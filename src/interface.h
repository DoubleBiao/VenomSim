#include "quadcopter.h"

extern "C" void siminit
	(double initx, double inity, double initz, double initroll, double initpitch, double inityaw,
	 double initx_t, double inity_t, double initz_t, double initroll_t, double initpitch_t, double inityaw_t);


extern "C" void simcontrol(double roll, double pitch, double yaw, char throttle,
												char roll_t, char pitch_t, char yaw_t, char throttle_t);

extern "C" void simrun(unsigned long long period);

extern "C" infoformat * siminfo();

extern "C" void simstop();
