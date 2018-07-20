#include <chrono>
#include <complex>
#include <math.h> 

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_state/conversions.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_test_restorebot");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

	const std::string PLANNING_GROUP = "Arm_Group";
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	/* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
	robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
	const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
	// Using the :moveit_core:`RobotModel`, we can construct a
	// :planning_scene:`PlanningScene` that maintains the state of
	// the world (including the robot).
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	// We will now construct a loader to load a planner, by name.
	// Note that we are using the ROS pluginlib library here.
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name; //"/ompl_interface/OMPLPlanner";

	namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("platform_base");

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script
       via buttons and keyboard shortcuts in RViz
       Currently, no remote control is being used */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "This is a test to display text", rvt::WHITE, rvt::XLARGE);

    ros::Publisher publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/display_robot_state/", 1);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    ros::Duration(2).sleep();

	collision_detection::CollisionRequest c_req;
	collision_detection::CollisionResult c_res;
    c_req.group_name = PLANNING_GROUP;
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;


    /*A
      3.01811
  1.01212
  1.17853
-0.580601
 -1.35777
  1.76285
  -0.253142
  	*/

	std::vector<double> start_joint_vals =  {-1.406,0.996909,1.14695,-1.5629,-2.21838,1.5244,-0.139682};
	robot_state->setJointGroupPositions(joint_model_group, start_joint_vals);  //maybe

    planning_scene->checkCollision(c_req, c_res, *robot_state);


    if (c_res.collision)
	{
	ROS_INFO_STREAM("A :: ");
	ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
	}
	else
	{
	ROS_INFO("Not colliding");
	}

	c_res.clear();
	moveit_msgs::RobotState robotStateMsg;
	moveit_msgs::DisplayRobotState displayRobotState;
	moveit::core::robotStateToRobotStateMsg(*robot_state, robotStateMsg);
	displayRobotState.state = robotStateMsg;
	publisher.publish(displayRobotState);
	visual_tools.prompt("Press next to continue");

	/*
	in between
	-0.643367
	  1.01536
	  1.18895
	-0.616549
	 -1.40424
	  1.79174
	-0.231444
	*/
	start_joint_vals = {-1.4555,1.00133,1.15971,-1.51599,-2.19185,1.57236,-0.128042};
	robot_state->setJointGroupPositions(joint_model_group, start_joint_vals);  //maybe

    planning_scene->checkCollision(c_req, c_res, *robot_state);

    if (c_res.collision)
	{
	ROS_INFO_STREAM("between :: ");
	ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
	}
	else
	{
	ROS_INFO("Not colliding");
	}

	moveit::core::robotStateToRobotStateMsg(*robot_state, robotStateMsg);
	displayRobotState.state = robotStateMsg;
	publisher.publish(displayRobotState);
	visual_tools.prompt("Press next to continue");

	c_res.clear();

	/*
	B
	 -3.08435
  1.01753
  1.19589
-0.640515
 -1.43523
  1.81101
-0.216979
	*/

	start_joint_vals = 	{-1.505,1.00576,1.17246,-1.46908,-2.16532,1.62033,-0.116402};
	robot_state->setJointGroupPositions(joint_model_group, start_joint_vals);  //maybe

    planning_scene->checkCollision(c_req, c_res, *robot_state);

    if (c_res.collision)
	{
	ROS_INFO_STREAM("B :: ");
	ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
	}
	else
	{
	ROS_INFO("Not colliding");
	}

	moveit::core::robotStateToRobotStateMsg(*robot_state, robotStateMsg);
	displayRobotState.state = robotStateMsg;
	publisher.publish(displayRobotState);
	visual_tools.prompt("Press next to continue");

	c_res.clear();
}


/*
A:
   3.08378
  0.947207
    1.2731
 -0.885475
  -1.44633
  -3.04449
-0.0285457

B:
  -3.02806
  0.961892
   1.27694
 -0.901836
  -1.51114
   3.07602
-0.0244677

between:
 -0.132975
  0.954936
   1.27512
 -0.894086
  -1.48044
  0.176831
-0.0263994


*/