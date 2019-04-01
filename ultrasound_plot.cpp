#include<ros/ros.h> 
#include<std_msgs/Float64.h>
#include <fstream>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <cmath>

double x_gripper = 0;
double curr_x = 0.422555886052;
double curr_y = 0.157611859505;
double y_goal = 0;
double  start_x= 0.422555886052;
double end_x = 0.157611859505;
int current_row = 0;
int start_row = 0;
int count = 0;
double x_val = 0;
double y_val = 0;
double z_val = 0;


  

std::ofstream myfile;
void sensorCallback(const std_msgs::Float64& message_holder) 
{ 
    
if (message_holder.data < 18){
  double increment = (start_x - end_x)/25;
  if (x_gripper == start_x){

    x_val = start_x-(increment*count);
    y_val = y_goal;
    z_val = message_holder.data;
	
  }
  else if (x_gripper == end_x){

	  x_val = end_x+(increment*count);
    y_val = y_goal;
    z_val = message_holder.data;
  }
  count +=1;
  current_row+=1;
}
} 

void xCallback(const std_msgs::Float64& message_holder)
{
  x_gripper = message_holder.data;
  //start_row = current_row
  //do csv stuff
  count = 0;
}

void yCallback(const std_msgs::Float64& message_holder)
{
  y_goal = message_holder.data;
}

int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"ultrasound_plot"); 

ros::NodeHandle n; 
  myfile.open("xyzData.csv");
  ros::Subscriber distZ_subscriber= n.subscribe("distMsg",1,sensorCallback); 
  ros::Subscriber distX_subscriber= n.subscribe("x_gripper_position",1,xCallback); 
  ros::Subscriber distY_subscriber= n.subscribe("y_gripper_position",1,yCallback);
  

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10); 
while (ros::ok){
//marker stuff for rviz
    visualization_msgs::Marker points, line_list;
    points.header.frame_id = line_list.header.frame_id = "/world";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "points_and_lines";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    points.scale.x = 0.2;
    points.scale.y = 0.2;
    
    line_list.scale.x = 0.1;

    points.color.g = 1.0f;
    points.color.a = 1.0;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p;

 while(true){
  p.x = x_val;
  p.y = y_val;
  p.z = z_val;
  ROS_INFO("plotting point: %f %f %f", x_val, y_val, z_val);
  points.points.push_back(p);
  line_list.points.push_back(p);
  p.z = 0;
  line_list.points.push_back(p);
  marker_pub.publish(points);
  marker_pub.publish(line_list);
}
}

  ros::spin();
  return 0; 
} 
