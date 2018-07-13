#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//some basic libraries for accessing information
#include <chrono>
#include <complex>
#include <math.h> 
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

//Tariq added libraries
#include <moveit/collision_distance_field/collision_detector_hybrid_plugin_loader.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/robot_state/conversions.h>



int main(int argc, char** argv)
{
	ros::init(argc, argv, "executor_planning_restorebot");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// BEGIN CODE
	//
	// Setup
	// ^^^^^
	//
	// MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
	// the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
	// are used interchangably.

	static const std::string PLANNING_GROUP = "Arm_Group";

	// The :move_group_interface:`MoveGroup` class can be easily
	// setup using just the name of the planning group you would like to control and plan for.
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	//Raw pointers are frequently used to refer to the planning group for improved performance
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()
		->getJointModelGroup(PLANNING_GROUP);

	ROS_INFO_STREAM("Entering visualization!");


	//VISUALIZATION
	//^^^^^^^^^^^^^
	//
	// The package MoveItVisualTools provides many capabilities for visualizing objects, robots
	// and trajectores in RViz as well as debugging tools such as step-by-step introspection
	// of a script
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("platform_base");
	visual_tools.deleteAllMarkers();

	//Remote control is an introspection tool that allows users to step through high level
	//script shorcuts via buttons and keyboard shortcuts in Rviz
	visual_tools.loadRemoteControl();

	//RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 3.14;
	visual_tools.publishText(text_pose, "Using Move_Group", rvt::WHITE, rvt:: XLARGE);

	//Batch publishing is used to reduce the number of messages being sent to RViz for large
	//visualizations
	visual_tools.trigger();

	//DISPLAYING BASIC INFORMATION
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//Print the name of the reference frame of the robot
	ROS_INFO_NAMED("move_group_executor::", "Reference frame: %s", move_group.getPlanningFrame().c_str());

	//Print the name of the end-effector link for this group
	ROS_INFO_NAMED("move_group_executor::", "Reference frame: %s", move_group.getEndEffectorLink().c_str());


	//Start Demo
	//^^^^^^^^^^
	//visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	//SET PLANNER
	int numFails = 0;
	double averageTime = 0.0;
	//^^^^^^^^^^^
	for(int over_iter = 0; over_iter < 100; over_iter++) 
	{
		// std::string planner_plugin_name;
		// node_handle.getParam("sbp_plugin", planner_plugin_name);
		// //TODO: import Planner correctly
		// move_group.setPlannerId(planner_plugin_name);
		//move_group.setPlannerId("FMTkConfigDefault");

		//PLAN A POSE GOAL
		//^^^^^^^^^^^^^^^^
		geometry_msgs::Pose target_pose;
		target_pose.orientation.w= 1.0;
		target_pose.position.x= 1.3306;
		target_pose.position.y = 2;
		target_pose.position.z = 3.64;
		//TODO: swap following code
		move_group.setPoseTarget(target_pose);
		//move_group.setApproximateJointValueTarget(target_pose);

		//Now, we call the planner to compute the plan and visualize it.
		//Note that we are just planning, not asking move_group
		//to actually move the robot
		moveit::planning_interface::MoveGroupInterface::Plan fmt_plan;

		bool success = (move_group.plan(fmt_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO_NAMED("move_group_executor::", "Visualizing the plan (pose goal) %s", success ? "Success" : "FAILED");

		//Visualizing plans
		//^^^^^^^^^^^^^^^^^
		//We can also visualize the plan as a line with markers in RViz.
		if(success) {
			averageTime += fmt_plan.planning_time_;
			ROS_INFO_NAMED("move_group_executor", "Visualizing FMT* plan as a trajectory line");
			visual_tools.publishAxisLabeled(target_pose, "fmt_pose");
			visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
			visual_tools.publishTrajectoryLine(fmt_plan.trajectory_, joint_model_group);
			visual_tools.trigger();
			//visual_tools.prompt("Press 'next' in the RVizVisualToolsGUI to continue with the demo (current WIP)");
		} else {
			numFails++;
		}


		// ////////////////////////////////
		// //SECOND PART of the CODE///////
		// ////////////////////////////////

		// //SET PLANNER
		// //^^^^^^^^^^^
		// //how access final state and trajectory from move_group?

		// move_group.setPlannerId("stomp");

		// //PLAN A JOINT SPACE GOAL
		// //^^^^^^^^^^^^^^^^^^^^^^^
		// //how obtain the joints of the final plan?
		// //does this work?
		// //TODO: Fix this; it currently sets an empty goal
		// move_group.setJointValueTarget(move_group.getJointValueTarget());  

		// //TODO: set SEED trajectory
		// trajectory_msgs::JointTrajectory seed = fmt_plan.trajectory_.joint_trajectory;
		// moveit_msgs::TrajectoryConstraints trajCon;
		// const auto dof = seed.joint_names.size();

		// for(size_t i = 0; i < seed.points.size(); i++) {
		// 	moveit_msgs::Constraints c;

		// 	if(seed.points[i].positions.size() != dof)
		// 		throw std::runtime_error("All trajectory position fields must have same dimensions as joint_names ");

		// 	for(size_t j = 0; j < dof; j++) { //for each joint
		// 		moveit_msgs::JointConstraint jc;
		//         jc.joint_name = seed.joint_names[j];
		// 		jc.position = seed.points[i].positions[j];

		// 		c.joint_constraints.push_back(jc);
		// 	}
		// 	trajCon.constraints.push_back(c);
		// }

		// move_group.setTrajectoryConstraints(trajCon);

		// moveit::planning_interface::MoveGroupInterface::Plan stomp_plan;
		// success = (move_group.plan(stomp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		// ROS_INFO_NAMED("move_group_executor::", "Visualizing the STOMP plan (pose goal) %s", success ? "Success" : "FAILED");

		// //Visualize the plan in RViz
		// visual_tools.deleteAllMarkers();
		// ROS_INFO_NAMED("move_group_executor", "Visualizing STOMP plan as a trajectory line");
		// visual_tools.publishText(text_pose, "STOMP Goal", rvt::WHITE, rvt::XLARGE);
		// visual_tools.publishTrajectoryLine(stomp_plan.trajectory_, joint_model_group);
		// visual_tools.trigger();
		// // visual_tools.prompt("Finished the demo");
		ROS_INFO_STREAM("iteration number:: ");
		ROS_INFO_STREAM(over_iter);
	}
	ROS_INFO_STREAM("Number of Failed attempts");
	ROS_INFO_STREAM(numFails);

	averageTime = averageTime / (100-numFails);
	ROS_INFO_STREAM("Average Time spent on Plan (assuming successful)");
	ROS_INFO_STREAM(averageTime);
}