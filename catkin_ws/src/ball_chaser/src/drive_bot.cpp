#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"

class BallChaser
{
public:
  BallChaser()
  { 
     // The service server
     server = n.advertiseService("/ball_chaser/command_robot", &BallChaser::handle_drive_request, this); 

    // what we are publishing -> to robots actuation topic
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

   ROS_INFO("Ready to send command velocities");

  }

  //Takes requested linear and angular speed and publish them
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){

    // acknowledge the information recieved
    ROS_INFO("DriveToTarget request recieved - linear.x:%1.2f, angular.x:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // create the geometry twist [6 vector]
    geometry_msgs::Twist move_;
    move_.linear.x = req.linear_x;
    move_.angular.z = req.angular_z;

    // publish move
    motor_command_publisher.publish(move_);
    
    // return response message
    res.msg_feedback = "Velocity set -linear.x: " + std::to_string(req.linear_x) + ", angular.z: "+ std::to_string(req.angular_z);

     // pass the stream
     ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }

private:
  ros::NodeHandle n; 
  ros::Publisher motor_command_publisher;
  ros::ServiceServer server;
 
};

int main(int argc, char** argv){

  // intialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // call the pusub class
  BallChaser ballChaser;

  // Handle ROS communication events
  ros::spin();

  return 0;
  
}
