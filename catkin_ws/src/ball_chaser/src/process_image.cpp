#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage
{
  public:
    ProcessImage()
    { 
      // client requesting from command_robot -> drive_bot
      client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

      // subscribing to the image topic
      camera_sub = n.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);

    }

    // functions
    void drive_robot(const float lin_x, const float ang_z){
      
      // Request a service and pass the velocities to it to drive the robot
      ROS_INFO_STREAM("Moving the vehicle towards the ball");
      
      ball_chaser::DriveToTarget move_;
      move_.request.linear_x = lin_x;
      move_.request.angular_z = ang_z;

      // call the safe_move service and pass the requested velocity changes
      if (!client.call(move_))
        ROS_ERROR("Failed to call the ball_chaser/command_robot service");
    }

    void process_image_callback(const sensor_msgs::Image img){
    
      int white_pixel = 255;
      int j_location = 0;
      bool ball_found = false;
      bool first_edge = false;
      int mid_ = img.step*0.5;
      
      // Loop through the image
      for (int i=0; i<img.height; i++){
        for (int j=0; j<img.step; j+=3){
             
             // check if all three channels sum up to white
             bool r_ = img.data[i*img.step + j] == 255;
             bool g_ = img.data[i*img.step + j + 1] == 255;
             bool b_ = img.data[i*img.step + j + 2] == 255;
       
            // check for white ball
            if (r_ && g_ && b_){               
               j_location = (i==0 && j==0)? j : 0.5*(j_location + j);
               ball_found = true;
               first_edge = true;
            }
            
            // check if other side has been reached
            if (first_edge && img.data[i*img.step + j + 1] != white_pixel){
               // set first_edge to false and break from loop
               first_edge = false;
               break;
              }               
         
        }
     }
     
     // covert to fractions
     float point_move = (float)j_location / (float)img.step;
     
          
     // calculating movement direction
     if (ball_found && fabs(point_move - 0.5) <= 0.05){
       
       // pretty much centered
          drive_robot(3.142, 0);

     } else if(point_move < 0.5 && ball_found){
       
        // left turn
        float turn = fabs((float)j_location - (float)mid_)/(float)mid_;
        if (point_move < 0.3){
           // too far left
           drive_robot(0, 3.142*turn);

        } else { drive_robot(turn, 3.142*turn);}
      
     } else if (point_move > 0.5 && ball_found){
       
        // right turn
        float turn = fabs((float)j_location - (float)mid_)/(float)mid_;
        if (point_move > 0.7){
           // too far right
           drive_robot(0, -3.142*turn);

        } else { drive_robot(turn, -3.142*turn);}
      
     } else {
        
       // don't move
       drive_robot(0, 0);
     }
     
    } 
     
  private:
  ros::NodeHandle n; 
  ros::Subscriber camera_sub;
  ros::ServiceClient client;
 
};

int main(int argc, char** argv){
  // initiate ROS
  ros::init(argc, argv, "process_image");

  // create the process image
  ProcessImage image_proc;

  // Handle ROS communication events
  ros::spin();

  return 0;
}
  
