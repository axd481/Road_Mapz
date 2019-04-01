

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

double start_y;
double start_x;
double end_x;
std::vector<double> z_arr;
double z_sum;
double z_avg;
std_msgs::Float64 sensorData;

//callback function for the arduino subscriber
void arduinoCallback(const std_msgs::Float64& distance_msg)
{
    sensorData.data = distance_msg.data;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "Move_140_raster"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   

    //defining subscriber of arduino topic
    ros::NodeHandle n;
    ros::Subscriber arduino_sub_object = n.subscribe("distMsg", 1, arduinoCallback);

    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    int rtn_val;
    int nsteps;
    double arrival_time;
    double arrival_time_y;
    double bible_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;
    int rtn_code;
    double x;
    double y;
    nsteps = 10;
    arrival_time = 2.0;
    arrival_time_y = 0.5;
    bible_time = 0.5;


    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    ros::init(argc, argv, "position_publisher"); // name of this node will be "minimal_publisher"
    ros::Publisher my_position_object_x = n.advertise<std_msgs::Float64>("x_gripper_position", 1);
    ros::Publisher my_position_object_y = n.advertise<std_msgs::Float64>("y_gripper_position", 1);
    std_msgs::Float64 pos_float; //create a variable of type "Float64", 
	Eigen::Affine3d tool_affine;
    while (ros::ok()) {

	//Move robot to gripper down position first before anything
	b_des << 1,0,0;
        n_des << 0,0,-1;
        t_des << 0,1,0;

    	R_gripper.col(0) = n_des;
    	R_gripper.col(1) = t_des;
    	R_gripper.col(2) = b_des;

    	O_des << 0.515000 + 0.20, 0.0, 0.711999;
    	tool_affine.linear() = R_gripper;
    	tool_affine.translation() = O_des;
    	tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
	//Move to home position 
	 ROS_INFO("Moving to home position");      
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        b_des << 0, 0, -1;
        n_des << -1, 0, 0;
        t_des = b_des.cross(n_des);

        R_gripper.col(0) = n_des;
        R_gripper.col(1) = t_des;
        R_gripper.col(2) = b_des;

        O_des << 0.45,0.3,0.367;
        tool_affine.linear() = R_gripper;
        tool_affine.translation() = O_des;
        tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
        ROS_INFO("requesting plan to gripper-down pose:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	start_x= 0.422555886052;
	end_x = 0.157611859505;
	start_y = 0.518852834952;
	tool_pose.pose.position.x = 0.422555886052; 
	tool_pose.pose.position.y = 0.518852834952; 
	tool_pose.pose.position.z = 0.25; 
	

        ROS_INFO("moving to starting position");
	
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	while(start_y < 0.70){
		ROS_INFO("Starting raster pattern");
        pos_float.data = end_x;
        my_position_object_x.publish(pos_float);
        y = start_y;
        pos_float.data = y; // publish the value--of type Float64-- 
        
        my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 
		tool_pose.pose.position.x = end_x;
		xformUtils.printPose(tool_pose);
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}  
	
		ROS_INFO("MOVING Y MOVING Y MOVING Y");
        start_y+=.02;
		tool_pose.pose.position.y = start_y;
		xformUtils.printPose(tool_pose);
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time_y, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}    


		y = start_y;
		pos_float.data = y; // publish the value--of type Float64-- 
		my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 
		
        pos_float.data = start_x;
        my_position_object_x.publish(pos_float);

		tool_pose.pose.position.x = start_x;
		xformUtils.printPose(tool_pose);
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}  

		ROS_INFO("MOVING Y MOVING Y MOVING Y");
        start_y+=.02;
		tool_pose.pose.position.y = start_y;
		xformUtils.printPose(tool_pose);
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time_y, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}    

		y = start_y;
		pos_float.data = y; // publish the value--of type Float64-- 
		my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 
    }
	}
	
	ROS_INFO("Completed raster");
	ros::Duration(1.0).sleep();
	//Moves robot to home position
	b_des << 1,0,0;
        n_des << 0,0,-1;
        t_des << 0,1,0;

    	R_gripper.col(0) = n_des;
    	R_gripper.col(1) = t_des;
    	R_gripper.col(2) = b_des;
	
    	O_des << 0.515000 + 0.20, 0.0, 0.711999;
    	tool_affine.linear() = R_gripper;
    	tool_affine.translation() = O_des;
    	tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
	//Move to home position 
	 ROS_INFO("Moving to home position");      
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

	    return 0;
    }


