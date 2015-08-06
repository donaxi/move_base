#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include "phidgets_encoders/CountsXMotor.h"


class MotorControl
{
public:
MotorControl();

private:
void velCallback(const geometry_msgs::Twist::ConstPtr& Twist);
void countCallback(const phidgets_encoders::CountsXMotor::ConstPtr& Count);
void accelCallback(const geometry_msgs::Vector3::ConstPtr& Accel);
void orientCallback(const geometry_msgs::Vector3::ConstPtr& Orient);

double a_, b_, m1_target, m2_target, m3_target, m4_target, m1_vel, m2_vel, m3_vel, m4_vel;

ros::NodeHandle nh_;

ros::Subscriber get_vel;
ros::Subscriber get_count;
ros::Subscriber get_accel;
ros::Subscriber get_orient;

};

MotorControl::MotorControl():
{
	get_vel = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &MotorControl::velCallback, this);
	get_count = nh_.subscribe<phidgets_encoders::CountsXMotor>("wheel_count", 10, &MotorControl::countCallback, this);
	get_accel = nh_.subscribe<geometry_msgs::Vector3>("sensor_accel", 10, &MotorControl::accelCallback, this);
	get_orient = nh_.subscribe<geometry_msgs::Vector3>("sensor_orient", 10, &MotorControl::orientCallback, this);

}


void MotorControl::velCallback(const geometry_msgs::Twist::ConstPtr& Twist)
{
	float x_vel, y_vel, w_vel;

	x_vel = Twist->linear.x;
	y_vel = Twist->linear.y;
	w_vel = Twist->angular.z;

	m1_target = 50+70*((y_vel - x_vel + w_vel*(a_+b_))+1);
	m2_target = 50+70*(-(y_vel + x_vel - w_vel*(a_+b_))+1);
	m3_target = 50+70*(-(y_vel - x_vel - w_vel*(a_+b_))+1);
	m4_target = 50+70*((y_vel + x_vel + w_vel*(a_+b_))+1);
}

void MotorControl::countCallback(const phidgets_encoders::CountsXMotor::ConstPtr& Encoder)
{
	int index, count, count_change, time_;


}

void MotorControl::accelCallback(const geometry_msgs::Vector3::ConstPtr& Accel)
{
	double x_accel, y_accel;

}

void MotorControl::orientCallback(const geometry_msgs::Vector3::ConstPtr& Orient)
{
	double x_rel_position, y_rel_position;

}



int main(int argc, char** argv){
	ros::init(argc, argv, "move_base");
	MotorControl motor_control;
	ros::spin();
}