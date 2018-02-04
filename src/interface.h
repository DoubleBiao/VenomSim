#include "quadcopter.h"

extern "C" _declspec(dllexport) void siminit
	(double initx, double inity, double initz, double initroll, double initpitch, double inityaw,
	 double initx_t, double inity_t, double initz_t, double initroll_t, double initpitch_t, double inityaw_t);


extern "C" _declspec(dllexport) void simcontrol(char roll, char pitch, char yaw, char throttle,
												char roll_t, char pitch_t, char yaw_t, char throttle_t);

extern "C" _declspec(dllexport) void simrun(unsigned long long period);

extern "C" _declspec(dllexport) infoformat * siminfo();

extern "C" _declspec(dllexport) void simstop();