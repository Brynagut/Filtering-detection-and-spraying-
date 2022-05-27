/*
*author: Abhi alias Faize. 
*email: info@aiinspired.com
******* abhijith.anjana@gmail.com
*/
/* Explanation
* Have a better understanding about the Coordinate Frame Conventions for camera :http://www.ros.org/reps/rep-0103.html
* Now consider a point inside the point cloud and imagaine that point is formed on a XY plane where the perpendicular 
* distance from the plane to the camera is Z. 
* The perpendicular drawn from the camera to the plan hits at center of the XY plane 
* Also now we have the x and y coordinate of the point which is formed on the XY plane.
* Now X is the horizontal axis and Y is the vertical axis
*                             
*
*
*
*
*                                  X
*                    _____________________________  
*                   |                             |
*                   |                     *______________Z______________________
*                   |                             |  
*                   |             C|______________________Z_____________________|
*                  Y|              |              |                             | 
*                   |                             | 
*                   |                             | 
*                   |_____________________________|  
*
*
* C is the center of the plane which is Z meter away from the center of camera and * is the point on the plan
* Now we know Z is the perpendicular distance from the point to the camera. 
* If you need to find the  actual distance d from the point to the camera, you shloud calculate the hypotenuse hypot(pt.z, pt.x)
* Angle convention
* to the left of the camera is 0 degree and to the right is 180 degree. Like you stretch both the hands towards sides (deals with X axis). left 0-----|-----180 right (could be other way around, dont have sensor to test)
* to the  bottom of the camera is 0 degree and to the top is 108 degree. 
* angle the point made horizontally min_angle_radx=atan2(pt.z,pt.x);
* angle the point made Verticlly    min_angle_rady=atan2(pt.z, pt.y);
*/



#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <stdio.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "find_target/target_position.h"



tf2_ros::Buffer tf_buffer;



using namespace::std;

// find base link and camera position
const std::string from_frame = "camera_depth_optical_frame";
const std::string to_frame = "base_link";

geometry_msgs::Point target_position_base_frame;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



/* The function takes the 3D point and the frames in between which we want to transform. 
The transform function requires a pose of type geometry_msgs::PoseStamped (similar to a point object but with additional information) as input.

We simply construct input_pose_stamped out of the 3D point by adding information about the frame and the time. 
Then we can call the transform function of the tf_buffer object we created before. It takes the constructed input_pose_stamped, 
the frame we want to transform to and some timeout value. As a result we get the transformed output pose, also of type geometry_msgs::PoseStamped, 
from which we return only the position. */

geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) {
    
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));
  return output_pose_stamped.pose.position;
}  



void callback(const PointCloud::ConstPtr& msg){
  double minDistance=0.0;
  double min_angle_radx=0.0;
  double min_angle_rady=0.0;
  double xX=0.0,yY=0.0,zZ=0.0;
  int count=0;
  // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){//to iterate trough all the points in the filtered point cloud published by publisher
    if(atan2(pt.z, pt.y)*(180/3.14159265358979323846)>80.00){// atan2(z,y)= arctan(z/y) if z>0;
      // truncating points with less that 80 degree vertical angle
      // because the point formed could be ground. 
        if(count==0){
        // initializing the first point read as minimum distance point
        minDistance=hypot(pt.z, pt.x);
        min_angle_radx=atan2(pt.z,pt.x);
        min_angle_rady=atan2(pt.z, pt.y);
        xX=pt.x;
        yY=pt.y;
        zZ=pt.z;
        count++;
        }
       else if(hypot(pt.z, pt.x)<minDistance){
            // keep updating the minimum Distant point
            minDistance=hypot(pt.z, pt.x);
            min_angle_radx=atan2(pt.z,pt.x);
            min_angle_rady=atan2(pt.z, pt.y);
            xX=pt.x;
            yY=pt.y;
            zZ=pt.z;
        }
        else{
          continue;
        }
      }
  }


// uncomment if you want to display data from camera frame
/*
 ROS_INFO_STREAM("Distance="<<minDistance<<"\n");
 ROS_INFO_STREAM("Angle in Degree X axis="<<min_angle_radx*(180/3.14159265358979323846)<<"\n");
 ROS_INFO_STREAM("Angle in Degree Y axis="<<min_angle_rady*(180/3.14159265358979323846)<<"\n");
 ROS_INFO_STREAM("pointXcoordinate="<<xX<<"\n");
 ROS_INFO_STREAM("pointYcoordinate="<<yY<<"\n");
 ROS_INFO_STREAM("pointZcoordinate="<<zZ<<"\n");
 sleep(1); */



// To clean it up I reference the point from which the camear sees as p
 geometry_msgs::Point p;
  p.x = xX;
  p.y = yY;
  p.z = zZ;

    
  target_position_base_frame = transform_between_frames(p, from_frame, to_frame);

 
  ROS_INFO_STREAM("3d bush position base frame: x " << target_position_base_frame.x << " y " << target_position_base_frame.y << " z " << target_position_base_frame.z);
  sleep(1);
} 



// We want to create a service call response to instruct the robot hand
bool get_target_position(find_target::target_position::Request  &req,
    find_target::target_position::Response &res) {
      res.target_position = target_position_base_frame;
      return true;
    } 



int main(int argc, char** argv)
{
  ros::init(argc, argv,"sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<PointCloud>("bush_detected", 1, callback);

  tf2_ros::TransformListener listener(tf_buffer);

  ros::ServiceServer service = n.advertiseService("target_position",  get_target_position);

  ros::spin();
}
