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

#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>

#include "config.h"
#include "mathHelp.h"
#include "quadcopter.h"
#include "utils_diffequation.h"
#include "receiver.h"
#include "barometer.h"
#include "magnetometer.h"
#include "gyroscope.h"
#include "accelerometer.h"
#include "sensorFusion.h"
#include "stabilizer.h"
#include "esc_motor.h"


#include <iostream>

class quadcopter::quadcopterImpl
{
	public:

		quadcopterImpl();
		~quadcopterImpl();


		// extract attitude and other data of quadcopter
		double get_position(int index);
		double get_speed(int index);
		double get_attitude(int index);
		double get_motor_rpm(int index);
		double get_up_vector(int index);
		double get_direction_vector(int index);
		double get_acc(int index);
		double get_thrust();
		// check if simulation is still running
		bool startSimulation(double initx, double inity, double initz, double initroll, double initpitch, double inityaw);

		void dosimulating( QS_TIMER_TIME_TYPE period);
		void getcommands(double roll, double pitch, double yaw, double throttle);

		// source: http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:

		/* 
		 * These vectors describe the exact state of the quadcopter. They are advanced with every single timestep.
		 * They should not be used by the controller directly because that would mean perfect knowledge of the quadcopter.
		 * Shared values are used as mutexed variables for sharing with other threads.
		 *
		 */
		Vector3d x;				// quadcopter: x position, theta angle
		Vector3d xdot;
		Vector3d xdotdot;
		Vector3d theta;
		Vector3d thetadot;
		Vector4d pwmDutyCycle;				// motor
		Vector4d rpm;



		/*
		 * For the renderer it is more convinient to use up- and direction-vector.
		 * The direction vector is the vector (1 0 0) in the body frame transformed into the earth frame.
		 * The up vector is the vector (0 0 1) in the body frame transformed into the earth frame.
		 * Shared values are used as mutexed variables for sharing with other threads.
		 *
		 */
		Vector3d up_vector, up_vector_shared;
		Vector3d direction_vector, direction_vector_shared;



		/* 
		 * This variables define the quadcopter mechanics and electronics. Frame mode is x/h.
		 * 
		 */
		int frame_mode;
		Matrix3d Inertia;		
		esc_motor escMotor0;
		esc_motor escMotor1;
		esc_motor escMotor2;
		esc_motor escMotor3;
		


		/*
		 * These values are send by the user. They describe the desired state of the quadcopter.
		 * For user input the keyboard is used.
		 * If there is no input, the quadcopter should hold position.
		 * 
		 */		
		receiver receiv;
		Vector3d theta_user;
		double throttle_user; 
		


		/*
		 * These values will be read from the sensors.
		 * The sensors take the ideal attitude and velocity values from above, transform them if needed and add noise and offsets.
		 * 
		 */
		accelerometer accel;
		gyroscope gyros;
		magnetometer magne;
		barometer barom;
		Vector3d xdotdot_bf_sensor;			// from accelerometer (accelerations along x, y and z axis in the body frame (bf))
		Vector3d thetadot_bf_sensor;		// from gyroscope (angular velocities in the body frame)
		Vector3d magneticField_bf_sensor;	// from magnetometer
		double height_barom_sensor;			// from barometer (height is the same in the body and earth frame)



		/*
		 * These values will be read from the sensor fusion.
		 * Data refers to the earth frame (ef).
		 * 
		 */
		sensorFusion sf;
		Vector3d theta_ef_sensor_fusion;
		Vector3d thetadot_ef_sensor_fusion;
		double height_ef_sensor_fusion;
		double heightdot_ef_sensor_fusion;
		double heightdotdot_ef_sensor_fusion;



		/*
		 * The stabilizer outputs the pwm duty cycle.
		 * 
		 */
		stabilizer stabi;



		// functions to manipulate quadcopter position
		void setZeroState();
		void setInitState(double initx, double inity, double initz, double initroll, double initpitch, double inityaw);

		// startup functions
		void calibrate_sensors();
		void attitute_height_estimation_before_takeoff();

		// functions to use the subelements
		void read_receiver(double roll, double pitch, double yaw, double throttle);
		void read_sensors();
		void read_sensorFusion();
		void read_controller();

		// function to advance simulation
		void solve_diff_equation(QS_TIMER_TIME_TYPE time_delta_simulation, QS_TIMER_TIME_TYPE period);

		// actual loop for simulation
		

		// simulation statistics
		unsigned long long iterations;
		unsigned long long total_time;
};

quadcopter::quadcopterImpl::quadcopterImpl()
{
	// init frame mode
	this->frame_mode = QS_FRAME_MODE_DEFAULT;

	// initialize thread variables and statistics
	this->iterations = 0;
	this->total_time = 0;

	// instanciate thread variable

	// set random seed
	srand(static_cast<unsigned int>(time(0)));

	// open console for debugging if defined
#ifdef QS_DEBUG_WITH_CONSOLE
	AllocConsole();
	AttachConsole(GetCurrentProcessId());
	freopen("CON","w",stdout);
#endif

	// print data for user orientation
#ifdef QS_DEBUG_WITH_CONSOLE
	printf("*********************************************\n");
	printf("pwms (esc):\n");
	printf("pwm min: %f Duty Cycle\n", MOTOR_ESC_PWM_MIN);
	printf("pwm max: %f Duty Cycle\n", MOTOR_ESC_PWM_MAX); 
	printf("pwm equ: %f Duty Cycle\n\n", getPWMinPointOfEquilibirum());
	printf("mass:\n");
	printf("mass: %f kg\n\n", MASS); 
	printf("thrusts:\n");
	printf("thrust min: %f N\n", MOTOR_THRUST_MIN);
	printf("thrust max: %f N\n", MOTOR_THRUST_MAX); 
	printf("thrust equ: %f N\n\n", MOTOR_THRUST_EQU); 
	printf("rpms:\n");
	printf("motor rpm min: %f rpm\n", MOTOR_RPM_MIN);
	printf("motor rpm max: %f rpm\n", MOTOR_RPM_MAX); 
	printf("motor rpm equ: %f rpm\n\n", MOTOR_RPM_EQU); 
#endif
}

quadcopter::quadcopterImpl::~quadcopterImpl()
{
	
}


double quadcopter::quadcopterImpl::get_position(int index)
{
	double temp;
	temp = this->x(index);

	return temp;
}

double quadcopter::quadcopterImpl::get_speed(int index)
{
	double temp;
	temp = this->xdot(index);

	return temp;
}

double quadcopter::quadcopterImpl::get_acc(int index)
{
	double temp;
	temp = this->xdotdot(index);

	return temp;
}

double quadcopter::quadcopterImpl::get_attitude(int index)
{
	double temp;
	temp = this->theta(index);
	if(index == 2)  //yaw 
		temp = wrap_Pi(temp);
	return temp;
}

double quadcopter::quadcopterImpl::get_motor_rpm(int index)
{
	double temp;
	temp = this->rpm(index);

	return temp;
}

double quadcopter::quadcopterImpl::get_thrust()
{
	return thrust_scalar(this->rpm,MOTOR_CONSTANT);
}

double quadcopter::quadcopterImpl::get_up_vector(int index)
{
	double temp;
	temp = this->up_vector(index);

	return temp;
}
 
double quadcopter::quadcopterImpl::get_direction_vector(int index)
{
	double temp;
	temp = this->direction_vector(index);

	return temp;
}

bool quadcopter::quadcopterImpl::startSimulation(double initx, double inity, double initz, double initroll, double initpitch, double inityaw)
{


	this->calibrate_sensors();
	this->setInitState(initx, inity,  initz,  initroll,initpitch, inityaw);
	this->attitute_height_estimation_before_takeoff();

	// init stabilizer
	this->stabi.setInitYawLock(this->theta_ef_sensor_fusion(2));
	this->stabi.setInitHeightLock(this->height_ef_sensor_fusion);

	return true;
}

void quadcopter::quadcopterImpl::setZeroState()
{
	this->x = Vector3d(0.0, 0.0, 0.0);
	this->xdot = Vector3d(0.0, 0.0, 0.0);	
	this->xdotdot = Vector3d(0.0, 0.0, 0.0);
	this->theta = Vector3d(0.0, 0.0, 0.0);
	this->thetadot = Vector3d(0.0, 0.0, 0.0);

	this->pwmDutyCycle(0) = 0.0;
	this->pwmDutyCycle(1) = 0.0;
	this->pwmDutyCycle(2) = 0.0;
	this->pwmDutyCycle(3) = 0.0;
	
	this->rpm(0) = 0.0; 
	this->rpm(1) = 0.0; 
	this->rpm(2) = 0.0; 
	this->rpm(3) = 0.0;
	
#ifdef RECALCULATE_I
	calc_inert_mat(&this->Inertia, CENTRAL_MASS, CENTRAL_MASS_RADIUS, MOTOR_MASS, LENGTH_ARM);
#else
	this->Inertia <<	INERTIA_X, 0, 0,
						0, INERTIA_Y, 0,
						0, 0, INERTIA_Z;
#endif
}

void quadcopter::quadcopterImpl::setInitState(double initx, double inity, double initz, double initroll, double initpitch, double inityaw)
{
	// set the initial state of the quadcopter as defined in "config.h"
	this->x(0) = initx;//X_START; 
	this->x(1) = inity;//Y_START; 
	this->x(2) = initz;//Z_START;
	
	this->xdot = Vector3d(0.0, 0.0, 0.0);
	this->xdotdot = Vector3d(0.0, 0.0, 0.0);
	
	this->theta(0) = DEG2RAD(initroll);//ROLL_START); 
	this->theta(1) = DEG2RAD(initpitch);//PITCH_START), 
	this->theta(2) = wrap_Pi(DEG2RAD(inityaw));//YAW_START));
	
	this->thetadot = Vector3d(0.0, 0.0, 0.0);
	
#ifdef RECALCULATE_I
	calc_inert_mat(&this->Inertia, CENTRAL_MASS, CENTRAL_MASS_RADIUS, MOTOR_MASS, LENGTH_ARM);
#else
	this->Inertia <<	INERTIA_X, 0, 0,
						0, INERTIA_Y, 0,
						0, 0, INERTIA_Z;
#endif
}

void quadcopter::quadcopterImpl::calibrate_sensors()
{
	// bring the quadcopter into the default position
	this->setZeroState();

	// Sensor Calibration:
	// Accelerometer chip: horizontal position, calibrate so that only gravitation vector is shown
	// Magnetometer chip: horizontal position and directed to north, calibrate so yaw becomes zero
	// Gyroscope chip: is calibrated at each start up by sensor fusion
	// Barometer chip: does no need to be calibrated since absolute height is irrelevant
	magne.take_chip_off_quadcopter();
	accel.take_chip_off_quadcopter();
	gyros.take_chip_off_quadcopter();

	for(int i=0; i<MAGNETOMETER_CAL_READS; i++)
		this->magne.read_calibration_value(this->theta);
	for(int i=0; i<ACCELEROMETER_CAL_READS; i++)
		this->accel.read_calibration_value(this->xdotdot, this->theta);

	magne.calibrate();
	accel.calibrate();

	// Chip on Quadcopter offset calibration:
	// quadcopter: motors in horizontal position and quadcopter directed to north, calibrate the offset between sensor chips and quadcopter frame
	magne.place_chip_on_quadcopter();
	accel.place_chip_on_quadcopter();
	gyros.place_chip_on_quadcopter();
	do
	{
		this->read_sensors();
	} while(!sf.writeSensorTiltCalibData(this->xdotdot_bf_sensor, this->magneticField_bf_sensor));
}

void quadcopter::quadcopterImpl::attitute_height_estimation_before_takeoff()
{
	// As in reality, the quadcopter starts from the standstill, so no velocity and no acceleration (linear and angular).
	// From this position it can calculate it's initial attitude.
	// Since the accelerometer is calibrated, this estimation should be very good.
	// Often, the calibration attitude and the start position are almost the same.

	// read sensors until sensor fusion says it's done
	Vector3d init_accel_corrupted;
	Vector3d init_gyros_corrupted;
	Vector3d init_magne_corrupted;
	double init_height_corrupted;

	do
	{
		this->accel.get_corrupted_accelerations(&init_accel_corrupted, this->xdotdot, this->theta);
		this->gyros.get_corrupted_angveloc(&init_gyros_corrupted, this->thetadot, this->theta);
		this->magne.get_corrupted_MagneticVectorBodyFrame(&init_magne_corrupted, this->theta);
		this->barom.get_corrupted_height(&init_height_corrupted, this->x(2));

	}while(sf.writeInitData(init_accel_corrupted, init_gyros_corrupted, init_magne_corrupted, Vector3d(0.0, 0.0, init_height_corrupted)) != true);

	// extract data in earth frame
	this->theta_ef_sensor_fusion = sf.getRPY();
	this->thetadot_ef_sensor_fusion = sf.getRPYDot();
	this->height_ef_sensor_fusion = sf.getHeight();
	this->heightdot_ef_sensor_fusion = sf.getHeightDot();
	this->heightdotdot_ef_sensor_fusion = sf.getHeightDotDot();
}

void quadcopter::quadcopterImpl::read_receiver(double roll, double pitch, double yaw, double throttle)
{
	this->receiv.get_desired_throttle(this->throttle_user,throttle);
	this->receiv.get_desired_theta(this->theta_user,roll,pitch,yaw);
	std::cout<<this->throttle_user<<std::endl;
	//std::cout<<this->theta_user<<std::endl;
}

void quadcopter::quadcopterImpl::read_sensors(void)
{
	// give the sensors the ideal values, they return a realistic value with an offset and noise
	this->accel.get_corrupted_accelerations(&(this->xdotdot_bf_sensor), this->xdotdot, this->theta);
	this->gyros.get_corrupted_angveloc(&(this->thetadot_bf_sensor), this->thetadot, this->theta);
	this->magne.get_corrupted_MagneticVectorBodyFrame(&(this->magneticField_bf_sensor), this->theta);
	this->barom.get_corrupted_height(&(this->height_barom_sensor), this->x(2));



}

void quadcopter::quadcopterImpl::read_sensorFusion(void)
{
	// write new sensor data to sensor fusion
	this->sf.writeData(this->xdotdot_bf_sensor, this->thetadot_bf_sensor, this->magneticField_bf_sensor, Vector3d(0.0,0.0,this->height_barom_sensor));

	// extract data in earth frame
	this->theta_ef_sensor_fusion = sf.getRPY();
	this->thetadot_ef_sensor_fusion = sf.getRPYDot();
	this->height_ef_sensor_fusion = sf.getHeight();
	this->heightdot_ef_sensor_fusion = sf.getHeightDot();
	this->heightdotdot_ef_sensor_fusion = sf.getHeightDotDot();

	//std::cout<<this->theta<<std::endl<<std::endl;
	//std::cout<<this->theta_ef_sensor_fusion<<std::endl<<std::endl<<std::endl<<std::endl;

}

void quadcopter::quadcopterImpl::read_controller()
{	
	this->stabi.compute_pwmDutyCycle(	&(this->pwmDutyCycle),
						this->theta_ef_sensor_fusion,			// attitude
						this->thetadot_ef_sensor_fusion,		// angular velocity
						this->height_ef_sensor_fusion,			// height
						this->heightdot_ef_sensor_fusion,		// vertical speed
						this->heightdotdot_ef_sensor_fusion,	// vertical acceleration
						this->theta_user(0),					// user roll
						this->theta_user(1),					// user pitch
						this->theta_user(2),					// user yaw
						this->throttle_user);					// user thrust
}

void quadcopter::quadcopterImpl::solve_diff_equation(QS_TIMER_TIME_TYPE time_delta_simulation, QS_TIMER_TIME_TYPE period)
{
	// here we solve the equation for the quadcopter as a rigid body and the equations for the motors

	// refresh input signals for motors
	this->escMotor0.esc_set_inputPWMDutyCycle(this->pwmDutyCycle(0));
	this->escMotor1.esc_set_inputPWMDutyCycle(this->pwmDutyCycle(1));
	this->escMotor2.esc_set_inputPWMDutyCycle(this->pwmDutyCycle(2));
	this->escMotor3.esc_set_inputPWMDutyCycle(this->pwmDutyCycle(3));
	
	// convert from nanoseconds in integer to seconds in double
	double d_dt = ((double) time_delta_simulation) / ((double) 1e9);
	
	for(unsigned int i=0; i < period / time_delta_simulation; i++)
	{
		// ADVANCE EACH MOTOR:
		
		this->escMotor0.solve_diff_equation_step(&(this->rpm(0)), d_dt);
		this->escMotor1.solve_diff_equation_step(&(this->rpm(1)), d_dt);
		this->escMotor2.solve_diff_equation_step(&(this->rpm(2)), d_dt);
		this->escMotor3.solve_diff_equation_step(&(this->rpm(3)), d_dt);
		
		// ADVANCE RIGID BODY:
		
		// compute accelerations
		acceleration(&xdotdot, this->rpm, this->theta, this->xdot, MASS, GRAVITY, MOTOR_CONSTANT, Vector3d(DRAG_CONSTANT_X, DRAG_CONSTANT_Y, DRAG_CONSTANT_Z));
        
        // advance system state
        this->xdot = this->xdot + d_dt * xdotdot;
        this->x = this->x + d_dt * this->xdot;
        
        // compute angular accelerations
        Vector3d omega, omegadot;
        thetadot2omega(&omega, this->thetadot, this->theta);
		angular_acceleration(&omegadot, this->rpm, omega, this->Inertia, LENGTH_ARM, TORQUE_YAW_CONSTANT, MOTOR_CONSTANT, this->frame_mode);

        // advance system state
        omega = omega + d_dt * omegadot;
        omega2thetadot(&(this->thetadot), omega, this->theta);
        this->theta = this->theta + d_dt * this->thetadot;
	}
	
	// calculate up and direction vector
	Matrix3d R;
	rotation(&R, this->theta);
	Vector3d direction_vector_body(1,0,0);
	Vector3d up_vector_body(0,0,1);
	this->direction_vector = R*direction_vector_body;
	this->up_vector = R*up_vector_body;
	

	
	
	// statistics
	this->iterations += period/time_delta_simulation;
	this->total_time += period;
}

void quadcopter::quadcopterImpl::dosimulating( QS_TIMER_TIME_TYPE period)
{
	this->solve_diff_equation(QS_TIME_DELTA, period);
}

void quadcopter::quadcopterImpl::getcommands(double roll, double pitch, double yaw, double throttle)
{
	this->read_receiver(roll, pitch, yaw, throttle);
	this->read_sensors();
	this->read_sensorFusion();
	this->read_controller();
}

// =================================================================
// ====================== Interface class ==========================
// =================================================================
quadcopter::quadcopter()
{
	this->qcimpl = new quadcopterImpl();
}

quadcopter::~quadcopter()
{
	delete this->qcimpl;
	this->qcimpl = 0;
}



double quadcopter::get_position(int index)
{
	return this->qcimpl->get_position(index);
}

double quadcopter::get_speed(int index)
{
	return this->qcimpl->get_speed(index);
}

double quadcopter::get_attitude(int index)
{
	return this->qcimpl->get_attitude(index);
}

double quadcopter::get_motor_rpm(int index)
{
	return this->qcimpl->get_motor_rpm(index);
}

double quadcopter::get_up_vector(int index)
{
	return this->qcimpl->get_up_vector(index);
}

double quadcopter::get_acc(int index)
{
	return this->qcimpl->get_acc(index);
}

double quadcopter::get_direction_vector(int index)
{
	return this->qcimpl->get_direction_vector(index);
}

double quadcopter::get_thrust()
{
	return this->qcimpl->get_thrust();
}

bool quadcopter::startSimulation(double initx, double inity, double initz, double initroll, double initpitch, double inityaw)
{
	return this->qcimpl->startSimulation(initx,  inity,  initz, initroll, initpitch,inityaw);
}



void quadcopter::dosimulating( QS_TIMER_TIME_TYPE period)
{
	this->qcimpl->dosimulating(period);
}

void quadcopter::getcommands(double roll, double pitch, double yaw, double throttle)
{
	this->qcimpl->getcommands(roll, pitch, yaw, throttle);
}
