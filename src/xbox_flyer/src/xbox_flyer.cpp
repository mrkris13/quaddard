// xbox_flyer.cpp

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <mavros_msgs/AttitudeTarget.h>

class XboxFlyer
{
private:
  ros::NodeHandle _nh;

public:
  XboxFlyer();
  ~XboxFlyer();
};

XboxFlyer::XboxFlyer()
{
  _nh = ros::NodeHandle("~");

  return;
}

XboxFlyer::~XboxFlyer()
{
  return;
}








int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_flyer");

  XboxFlyer flyer;

  ros::spin();

  return 0;
}