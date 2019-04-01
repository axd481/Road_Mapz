#include <ros/ros.h> 
#include <std_msgs/Float64.h>
#include <fstream>
#include <iostream>


double x_gripper = 0;
double curr_x = 0.422555886052;
double curr_y = 0.157611859505;
double y_goal = 0;
double  start_x= 0.422555886052;
double end_x = 0.157611859505;
int current_row = 0;
int start_row = 0;
int count = 0;
  

std::ofstream myfile;
void sensorCallback(const std_msgs::Float64& message_holder) 
{ 
   
if (message_holder.data < 18){
  double increment = (start_x - end_x)/25;
  if (x_gripper == start_x){
  	myfile << start_x-(increment*count) << "," << y_goal << "," << message_holder.data << std::endl;
	
  }
  else if (x_gripper == end_x){
  	myfile << end_x+(increment*count) << "," << y_goal << "," << message_holder.data << std::endl;

  }
  count +=1;
  current_row+=1;
}
  ROS_INFO("received value is: %f",message_holder.data); 


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
  ros::init(argc,argv,"ultrasound_sub"); 

ros::NodeHandle n; 
  myfile.open("xyzData.csv");
  ros::Subscriber sensor_subscriber= n.subscribe("distMsg",1,sensorCallback); 
  ros::Subscriber x_goal_subscriber= n.subscribe("x_gripper_position",1,xCallback); 
  ros::Subscriber y_goal_subscriber= n.subscribe("y_gripper_position",1,yCallback);
  



  ros::spin();
  return 0; 
} 
