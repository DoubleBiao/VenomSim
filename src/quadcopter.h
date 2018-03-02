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

#ifndef QS_QUADCOPTER_H
#define QS_QUADCOPTER_H

#include "config.h"

class quadcopter
{
	public:
	
		quadcopter();
		~quadcopter();

	

		// functions to extract attitude and other data of quadcopter: index mapping is 0 -> x, 1 -> y, 2 -> z
		double get_position(int index);
		double get_speed(int index);
		double get_acc(int index);
		double get_thrust();
		double get_attitude(int index);
		double get_motor_rpm(int index);
		double get_up_vector(int index);
		double get_direction_vector(int index);

		// check if simulation is still running
		bool startSimulation(double initx, double inity, double initz, double initraw, double initpitch, double inityaw,double speed_upbound);
		
		void dosimulating( QS_TIMER_TIME_TYPE timestep, QS_TIMER_TIME_TYPE period);
		void getcommands(double roll, double pitch, double yaw, double throttle);
	private:

		// implementation class
		class quadcopterImpl;
		quadcopterImpl* qcimpl;
};

struct infoformat
{
	double posx;
	double posy;
	double posz;

	double velocityx;
	double velocityy;
	double velocityz;

	double accx;
	double accy;
	double accz;

	double thetax;
	double thetay;
	double thetaz;

	//double omegax;
	//double omegay;
	//double omegaz;

	double posx_t;
	double posy_t;
	double posz_t;

	double velocityx_t;
	double velocityy_t;
	double velocityz_t;

	double accx_t;
	double accy_t;
	double accz_t;

	double thetax_t;
	double thetay_t;
	double thetaz_t;

	double thrust;
};

#endif
