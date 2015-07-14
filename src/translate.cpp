#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
TeleopTurtle();
 
private:
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

ros::NodeHandle nh_;

   int x_, y_, w_;
   double x_s_, y_s_, w_s_;

   ros::Publisher vel_pub_;
   ros::Subscriber joy_sub_;
};

 TeleopTurtle::TeleopTurtle():
   y_(1),
   x_(0),
   w_(2),
   x_s_(),
   y_s_(),
   w_s_()
{
   nh_.param("x_control", x_, x_);
   nh_.param("y_control", y_, y_);
   nh_.param("angular_control", w_, w_);
   nh_.param("x_s_", x_s_, x_s_);
   nh_.param("y_s_", y_s_, y_s_);
   nh_.param("w_s_", w_s_, w_s_);

   vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   geometry_msgs::Twist twist;
   double un_x_,un_y_,un_w_;

   un_y_ = joy->axes[y_];
   un_x_ = joy->axes[x_];
   un_w_ = joy->axes[w_];


   if (un_y_ >= 0)
   {
      twist.linear.y = un_y_ * (1/0.4845);
   }
   if (un_y_ < 0)
   {
      twist.linear.y = un_y_ * (1/0.50593);
   }
   else
   if (un_x_ >= 0)
   {
      twist.linear.x = -un_x_ * (1/0.80671);
   }
   if (un_x_ < 0)
   {
      twist.linear.x = -un_x_ * (1/0.48443);
   }
   if (un_w_ >= 0)
   {
      twist.angular.z = un_w_*(1/0.9786);
   }
   if (un_w_ < 0)
   {
      twist.angular.z = un_w_*(1/0.2697);
   }
   
   
   vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "teleop_turtle");
   TeleopTurtle teleop_turtle;
   ros::spin();
 }