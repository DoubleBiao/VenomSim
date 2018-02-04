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


#include "config.h"
#include "mathHelp.h"
#include "receiver.h"

receiver::receiver()
{
	this->roll_pwm = RECEIVER_ROLL_KEY_PWM;
	this->pitch_pwm	= RECEIVER_PITCH_KEY_PWM;
	this->yaw_pwm = RECEIVER_YAW_KEY_PWM;
	this->throttle_pwm = RECEIVER_THROTTLE_KEY_PWM;

	this->output_blocked = false;
}

receiver::~receiver()
{
}

void receiver::get_desired_theta(Vector3d &theta_d,char _roll,char _pitch, char _yaw)
{


	// zero signals in case no input or blocked
	theta_d(0) = RECEIVER_PWM_ZERO_SIGNAL;
	theta_d(1) = RECEIVER_PWM_ZERO_SIGNAL;
	theta_d(2) = RECEIVER_PWM_ZERO_SIGNAL;

	// only roll and pitch
	if(this->output_blocked == false)
	{
		// roll (A and D)
		if(_roll == '+')
			theta_d(0) -= this->roll_pwm;
		else if(_roll == '-')
			theta_d(0) += this->roll_pwm;

		// pitch (W and S)
		if(_pitch == '+')
			theta_d(1) += this->pitch_pwm;
		else if(_pitch == '-')
			theta_d(1) -= this->pitch_pwm;

		// yaw (Q and E)
		if(_yaw == '+')
			theta_d(2) -= this->yaw_pwm;
		else if(_yaw == '-')
			theta_d(2) += this->yaw_pwm;
	}

	theta_d(0) = constrainn<double>(theta_d(0), RECEIVER_PWM_MIN, RECEIVER_PWM_MAX);
	theta_d(1) = constrainn<double>(theta_d(1), RECEIVER_PWM_MIN, RECEIVER_PWM_MAX);
	theta_d(2) = constrainn<double>(theta_d(2), RECEIVER_PWM_MIN, RECEIVER_PWM_MAX);

}

void receiver::get_desired_throttle(double &throttle,char _throttle)
{	

		// user has to put stick in hover position if he does not want to climb/sink
	throttle = getPWMinPointOfEquilibirum();

	// if block demand normal position
	if(this->output_blocked == false)
	{
		// climb (+)
		if(_throttle == '+')
			throttle += this->throttle_pwm;
		// sink (-)
		else if(_throttle == '-')
			throttle -= this->throttle_pwm;
	}

	throttle = constrainn<double>(throttle, RECEIVER_PWM_MIN, RECEIVER_PWM_MAX);

}

void receiver::block_receiver(bool blocked)
{
	this->output_blocked = blocked;
}


