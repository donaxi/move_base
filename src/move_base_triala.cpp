#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <serial/serial.h>
#include <string>

// POLOLU SERVO MASTER TAGS
const uint cmpct_target= 0x84;
const uint mini_target  =0xFF;
const uint cmpct_set_speed = 0x87;
const uint cmpct_set_accel = 0x89;


// MOTOR CHANNEL FOR EACH MOTOR AT POLOLU SERVO MASTER
const uint m1_ = 0x00;
const uint m2_ = 0x01;
const uint m3_ = 0x02;
const uint m4_ = 0x03;


class MotorControl
{
public:
MotorControl();

private:
void velCallback(const geometry_msgs::Twist::ConstPtr& Twist);
void accelCallback(const geometry_msgs::Vector3::ConstPtr& Accel);
void orientCallback(const geometry_msgs::Vector3::ConstPtr& Orient);
void Pololu_set_target(int mn_, int trgt);
void Pololu_set_speed();
void Pololu_set_accel();


double a_, b_, m1_target, m2_target, m3_target, m4_target, m1_vel, m2_vel, m3_vel, m4_vel;

serial::Serial serial_;
std::string serial_addr;

//ROS
ros::NodeHandle nh_;

ros::Subscriber get_vel;
ros::Subscriber get_count;
ros::Subscriber get_accel;
ros::Subscriber get_orient;
};

MotorControl::MotorControl():
serial_addr("/dev/ttyACM1")
{
	nh_.param("a",a_,a_);
	nh_.param("b",b_,b_);
	nh_.param("serial_port",serial_addr,serial_addr);
	
	try
	{
		ROS_INFO("setting up address");
		serial_.setPort(serial_addr);
		ROS_INFO("setting up baudrate");
		serial_.setBaudrate(9600);
		ROS_INFO("setting up Timeout");
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		serial_.setTimeout(to);
		ROS_INFO("opening port");
		serial_.open();
	}
	catch(serial::IOException& e)
	{
		ROS_INFO("UNABLE TO OPEN PORT");
	}

	if(serial_.isOpen()){
		ROS_INFO("SERIAL PORT INITIALIZED");
		Pololu_set_target(m1_,127);
		Pololu_set_target(m2_,127);
		Pololu_set_target(m3_,127);
		Pololu_set_target(m4_,127);
		Pololu_set_speed();
	}

	get_vel = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &MotorControl::velCallback, this);
	get_accel = nh_.subscribe<geometry_msgs::Vector3>("sensor_accel", 10, &MotorControl::accelCallback, this);
	get_orient = nh_.subscribe<geometry_msgs::Vector3>("sensor_orient", 10, &MotorControl::orientCallback, this);
}

void MotorControl::Pololu_set_target(int mn_, int trgt){
	char wr[] = {mini_target,mn_,trgt};
	serial_.write(wr);
}

void MotorControl::Pololu_set_speed(){
	char wr[] = {cmpct_set_speed,m1_,0x0C,0x01,cmpct_set_speed,m2_,0x0C,0x01,cmpct_set_speed,m3_,0x0C,0x01,cmpct_set_speed,m4_,0x0C,0x01};
	serial_.write(wr);
	ROS_INFO("CHANGE SPEED SET");
}
void MotorControl::Pololu_set_accel(){
	char wr[] = {cmpct_set_accel,m1_,0x00,0x00,cmpct_set_accel,m2_,0x00,0x00,cmpct_set_accel,m3_,0x00,0x00,cmpct_set_accel,m4_,0x00,0x00};
	serial_.write(wr);
	ROS_INFO("CHANGE ACCEL SET");
}

void MotorControl::velCallback(const geometry_msgs::Twist::ConstPtr& Twist)
{
	float x_vel, y_vel, w_vel;

	x_vel = Twist->linear.x;
	y_vel = Twist->linear.y;
	w_vel = Twist->angular.z;

	m1_target = 127+127*((y_vel - x_vel + w_vel*(a_+b_)));
	m2_target = 127+127*(-(y_vel + x_vel - w_vel*(a_+b_)));
	m3_target = 127+127*(-(y_vel - x_vel - w_vel*(a_+b_)));
	m4_target = 127+127*((y_vel + x_vel + w_vel*(a_+b_)));
	ROS_INFO("CHANGING TARGET");
	Pololu_set_target(m1_,m1_target);
	Pololu_set_target(m2_,m2_target);
	Pololu_set_target(m3_,m3_target);
	Pololu_set_target(m4_,m4_target);
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