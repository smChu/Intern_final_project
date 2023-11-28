#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define LINEAR_MAX 1.0
#define ANGULAR_MAX 1.0

class Joy_cmd_vel
{
public:
  Joy_cmd_vel();
  void operate();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  float target_linear;
  float target_angular;
  float current_linear;
  float current_angular;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

Joy_cmd_vel::Joy_cmd_vel() : target_linear(0.0), target_angular(0.0), current_linear(0.0), current_angular(0.0)
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_cmd_vel::joyCallback, this);
}

void Joy_cmd_vel::operate()
{
  geometry_msgs::Twist twist;

  target_linear = std::min<float>(std::max<float>(target_linear, -LINEAR_MAX), LINEAR_MAX);
  target_angular = std::min<float>(std::max<float>(target_angular, -ANGULAR_MAX), ANGULAR_MAX);

  twist.linear.x = target_linear;
  twist.angular.z = target_angular;

  vel_pub_.publish(twist);
}

void Joy_cmd_vel::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  target_linear = joy->axes[4];
  target_angular = joy->axes[1];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel");
  Joy_cmd_vel vel;

  ros::Rate loop_rate(33);
  while (ros::ok())
  {
    vel.operate();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}