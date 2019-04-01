// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

//launch with roslaunch irb140_description irb140.launch, which places a block at x=0.5, y=0
// launch action server: rosrun irb140_planner irb140_cart_move_as
// then run this node

//for manual gripper control,  rosrun cwru_sticky_fingers finger_control_dummy_node /sticky_finger/link6 false

#include<ros/ros.h>
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
//std_msgs::Float64 xLoc;
//std_msgs::Float64 yLoc;
//std_msgs::Float64 [] posArray;

//callback function for the arduino subscriber
void arduinoCallback(const std_msgs::Float64& distance_msg)
{
    sensorData.data = distance_msg.data;
    //xLoc.data = tool_pose.pose.position.x;
    //yLoc.data = tool_pose.pose.position.y;
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
    arrival_time_y = 0.2;
    bible_time = 0.2;

    std::ofstream myfile;
    myfile.open("xyzData.csv");
    //myfile << 3 << "," << 4 << "," << 5 << std::endl;

    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    ros::init(argc, argv, "position_publisher"); // name of this node will be "minimal_publisher"
    //ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_position_object_x = n.advertise<std_msgs::Float64>("x_gripper_position", 1);
    ros::Publisher my_position_object_y = n.advertise<std_msgs::Float64>("y_gripper_position", 1);
    std_msgs::Float64 pos_float; //create a variable of type "Float64", 

    while (ros::ok()) {
	//Move robot to gripper down position first before anything with translation of 0.450, -0.000, 0.367
        Eigen::Affine3d tool_affine;
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
	
	//Get kong dog toy centroid coordinates from opencv node

        //move to kong dog toy
        ROS_INFO("moving to starting position");
	
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time - 2.0).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        } 
	//Turn vacuum gripper on 
	//system("sudo usbrelay BITFT_1=1");        
	while(start_y < 0.70){
		start_x= 0.422555886052;
		end_x = 0.157611859505;
		//Move robot arm down to pick up toy
		ROS_INFO("Starting raster pattern");
		for(double i = start_x-(start_x/20.000); i > end_x; i-=(start_x/20.000)){
			tool_pose.pose.position.x = i;
			ROS_INFO("HELLO HELLO HELLO HELLO HELLO %.6f", i);
			xformUtils.printPose(tool_pose);
			rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, bible_time, tool_pose);
			if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
			    ROS_INFO("successful plan; command execution of trajectory");
			    rtn_val = cart_motion_commander.execute_planned_traj();
			    // ros::Duration(arrival_time - 2.0).sleep();
			} else {
			    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
			}  
			ros::Duration(0.5).sleep();
			x = i;
			pos_float.data = x;
       			my_position_object_x.publish(pos_float);
			y = start_y;
			pos_float.data = y; // publish the value--of type Float64-- 
       			my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 
			ros::spinOnce();

			for(int j = 0; j < 50; j++){
				ros::spinOnce();
				z_arr.push_back(sensorData.data);
				ros::Duration(0.001).sleep();
			}
			z_sum = 0;
			for (int k = 0; k < z_arr.size(); k++){
				if(z_arr[k] )
				z_sum+= z_arr[k];
			}
			for (int l = 0; l < z_arr.size(); l++){
				ROS_INFO("THE Z VALUE IS %.5f", z_arr[l]);
			}
			z_avg = z_sum/z_arr.size();

			myfile << x << "," << y << "," << z_avg << std::endl;
			z_arr.clear();
		}

		tool_pose.pose.position.x = end_x;
		xformUtils.printPose(tool_pose);
		ROS_INFO("MOVING TO ENDX");
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time_y, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		    // ros::Duration(arrival_time - 2.0).sleep();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}  
		x = end_x;
		pos_float.data = x;
		my_position_object_x.publish(pos_float);
		y = start_y;
		pos_float.data = y; // publish the value--of type Float64-- 
		my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 

		for(int j = 0; j < 50; j++){
			ros::spinOnce();
			z_arr.push_back(sensorData.data);
			ros::Duration(0.001).sleep();
		}
		z_sum = 0;
		for (int k = 0; k< z_arr.size(); k++){
			z_sum+= z_arr[k];
		}
		z_avg = z_sum/z_arr.size();
//for (int l = 0; l < z_arr.size(); l++){
		//		ROS_INFO("THE Z VALUE IS %.5f", z_arr[l]);
		//	}

		myfile << x << "," << y << "," << z_avg << std::endl;
		z_arr.clear();
		ROS_INFO("MOVING Y MOVING Y MOVING Y");
		tool_pose.pose.position.y = start_y+=.02;
		xformUtils.printPose(tool_pose);
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time_y, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		    //ros::Duration(arrival_time - 2.0).sleep();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}    

		x = end_x;
		pos_float.data = x;
		my_position_object_x.publish(pos_float);
		y = start_y;
		pos_float.data = y; // publish the value--of type Float64-- 
		my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 

		for(int j = 0; j < 50; j++){
			ros::spinOnce();
			z_arr.push_back(sensorData.data);
			ros::Duration(0.001).sleep();
		}
		z_sum = 0;
		for (int k = 0; k< z_arr.size(); k++){
			z_sum+= z_arr[k];
		}
		z_avg = z_sum/z_arr.size();
		//for (int l = 0; l < z_arr.size(); l++){
		//		ROS_INFO("THE Z VALUE IS %.5f", z_arr[l]);
		//	}

		myfile << x << "," << y << "," << z_avg << std::endl;
		z_arr.clear();
		//Ascend until z = 0.35
		for(double i = end_x+(end_x/20.000); i < start_x; i+=(start_x/20.000)){
			ROS_INFO("ELLO ELLO ELLO AY AY AY AY %.6f", i);
			tool_pose.pose.position.x = i;
			xformUtils.printPose(tool_pose);
			rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, bible_time, tool_pose);
			if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
			    ROS_INFO("successful plan; command execution of trajectory");
			    rtn_val = cart_motion_commander.execute_planned_traj();
			    //ros::Duration(arrival_time - 2.0).sleep();
			} else {
			    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
			}  
			x = i;
			pos_float.data = x;
       			my_position_object_x.publish(pos_float);
			y = start_y;
			pos_float.data = y; // publish the value--of type Float64-- 
       			my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 
			ros::spinOnce();

			for(int j = 0; j < 50; j++){
				ros::spinOnce();
				z_arr.push_back(sensorData.data);
				ros::Duration(0.001).sleep();
			}
			z_sum = 0;
			for (int k = 0; k< z_arr.size(); k++){
				z_sum+= z_arr[k];
			}
			for (int l = 0; l < z_arr.size(); l++){
				ROS_INFO("THE Z VALUE IS %.5f", z_arr[l]);
			}
			z_avg = z_sum/z_arr.size();

			myfile << x << "," << y << "," << z_avg << std::endl;
			z_arr.clear();
		}

		tool_pose.pose.position.x = start_x;
		xformUtils.printPose(tool_pose);
		ROS_INFO("START X START X START X");
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time_y, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		    // ros::Duration(arrival_time - 2.0).sleep();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}  
			x = start_x;
			pos_float.data = x;
			my_position_object_x.publish(pos_float);
			y = start_y;
			pos_float.data = y; // publish the value--of type Float64-- 
			my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 
			ros::spinOnce();
			for(int j = 0; j < 50; j++){
				ros::spinOnce();
				z_arr.push_back(sensorData.data);
				ros::Duration(0.001).sleep();
			}
			z_sum = 0;
			for (int k = 0; k< z_arr.size(); k++){
				z_sum+= z_arr[k];
			}
			for (int l = 0; l < z_arr.size(); l++){
				ROS_INFO("THE Z VALUE IS %.5f", z_arr[l]);
			}
			z_avg = z_sum/z_arr.size();

			myfile << x << "," << y << "," << z_avg << std::endl;
			z_arr.clear();
		ROS_INFO("MOVING Y MOVING Y MOVING Y");
		tool_pose.pose.position.y = start_y+=.02;
		xformUtils.printPose(tool_pose);
		rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time_y, tool_pose);
		if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
		    ROS_INFO("successful plan; command execution of trajectory");
		    rtn_val = cart_motion_commander.execute_planned_traj();
		    //ros::Duration(arrival_time - 2.0).sleep();
		} else {
		    ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
		}
		x = start_x;
		pos_float.data = x;
		my_position_object_x.publish(pos_float);
		y = start_y;
		pos_float.data = y; // publish the value--of type Float64-- 
		my_position_object_y.publish(pos_float); // publish the value--of type Float64-- 
		ros::spinOnce();		
		for(int j = 0; j < 50; j++){
			ros::spinOnce();
			z_arr.push_back(sensorData.data);
			ros::Duration(0.001).sleep();
		}
		z_sum = 0;
		for (int k = 0; k< z_arr.size(); k++){
			z_sum+= z_arr[k];
		}
		z_avg = z_sum/z_arr.size();
		myfile << x << "," << y << "," << z_avg << std::endl;
		z_arr.clear();
	}
	
	ROS_INFO("Completed raster");
	ros::Duration(1.0).sleep();
	//Moves robot to home position
	//Move tool flange back to home position first
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
	
	
    }
	myfile.close();      
    return 0;
}

