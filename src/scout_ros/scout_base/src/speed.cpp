#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
geometry_msgs::Twist speed;
void callback(geometry_msgs::TwistStamped msg)
{
  if(msg.twist.linear.x >= 0.8)
    msg.twist.linear.x = 0.8;
  if(msg.twist.angular.z >= 0.8)
    msg.twist.angular.z = 0.8;
  speed = msg.twist;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "speed");
    ros::NodeHandle n;
    ros::Subscriber sub =n.subscribe("/cmd_vel_raw",50,callback);
    ros::Publisher pub =n.advertise<geometry_msgs::Twist>("/cmd_vel",5);
    ros::Rate r(50);
    while(ros::ok())
    {
      pub.publish(speed);
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
