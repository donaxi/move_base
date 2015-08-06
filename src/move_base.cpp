#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidget21.h>

int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr);
int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr);
int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description);
int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value);

class MotorControl
{
public:
MotorControl();

private:
void velCallback(const geometry_msgs::Twist::ConstPtr& Twist);


double a_ , b_, start_vel_;
CPhidgetAdvancedServoHandle servo;

ros::NodeHandle nh_;
ros::Subscriber get_vel;

};

MotorControl::MotorControl():
a_(1.0),b_(1.0),start_vel_(120)
{

	nh_.param("a_",a_,a_);
	nh_.param("b_",b_,b_);
	nh_.param("start_vel_",start_vel_,start_vel_);

	int result;
	double curr_pos;
	const char *err;
	double minAccel, maxVel;

	
	CPhidgetAdvancedServo_create(&servo);

	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);

	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);

	CPhidget_open((CPhidgetHandle)servo, -1);

	ROS_INFO("Waiting for Phidget to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		ROS_INFO("Problem waiting for attachment: %s\n", err);
	}

	ROS_INFO("Setting up.....\n");

	for (int i = 0; i < 4; ++i)
	{
		ROS_INFO("Motor %d",i);
		CPhidgetAdvancedServo_getAccelerationMin(servo, i, &minAccel);
		CPhidgetAdvancedServo_setAcceleration(servo, i, minAccel);
		CPhidgetAdvancedServo_getVelocityMax(servo, i, &maxVel);
		CPhidgetAdvancedServo_setVelocityLimit(servo, i, maxVel/2);
		CPhidgetAdvancedServo_setPosition (servo, i, start_vel_);
		CPhidgetAdvancedServo_setEngaged(servo, i, 1);
	}
	ROS_INFO("Setting up finished\n");


	get_vel = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &MotorControl::velCallback, this);

}


void MotorControl::velCallback(const geometry_msgs::Twist::ConstPtr& Twist)
{
	float x_vel, y_vel, w_vel, m1_vel, m2_vel, m3_vel, m4_vel;

	x_vel = Twist->linear.x;
	y_vel = Twist->linear.y;
	w_vel = Twist->angular.z;

	m1_vel = 50+70*((y_vel - x_vel + w_vel*(a_+b_))+1);
	m2_vel = 50+70*(-(y_vel + x_vel - w_vel*(a_+b_))+1);
	m3_vel = 50+70*(-(y_vel - x_vel - w_vel*(a_+b_))+1);
	m4_vel = 50+70*((y_vel + x_vel + w_vel*(a_+b_))+1);

	CPhidgetAdvancedServo_setPosition (servo, 0, m1_vel);
	CPhidgetAdvancedServo_setPosition (servo, 1, m2_vel);
	CPhidgetAdvancedServo_setPosition (servo, 2, m3_vel);
	CPhidgetAdvancedServo_setPosition (servo, 3, m4_vel);
}

int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	ROS_INFO("%s %10d attached!\n", name, serialNo);

	return 0;
}

int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	ROS_INFO("%s %10d detached!\n", name, serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{
	ROS_INFO("Error handled. %d - %s\n", ErrorCode, Description);

	return 0;
}

int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
	ROS_INFO("Motor: %d > Current Position: %f\n", Index, Value);
	return 0;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "move_base");
	MotorControl motor_control;
	ros::spin();
}