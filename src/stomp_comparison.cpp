
/*
Motion_plannning_api_tutorial.cpp headers
*/
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

#include <moveit/collision_distance_field/collision_detector_hybrid_plugin_loader.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/robot_state/conversions.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning_restorebot");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

 // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel and a
  // PlanningScene. We will start by instantiating a `RobotModelLoader`_
  // object, which will look up the robot description on the ROS
  // parameter server and construct a :moveit_core:`RobotModel` for us
  // to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
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

  for(int iteration = 0; iteration < 50; iteration++) {

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name; //"/ompl_interface/OMPLPlanner";

    // We will get the name of planning plugin we want to loadix
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!node_handle.getParam("sbp_plugin", planner_plugin_name)) //TODO: fix this to sbp_plugin
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }

    //pray this works- setting planning time to 2 seconds see if FMTk works properly

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    

    //Ensure that the marker array panel (subscribed to /rviz_visual_tools) is active

    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("platform_base");


    // /*
    // original line: 
    //   moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    // There doesn't seem to be a panda_link0 joint that's moveable, so I'm going to assume that
    // it's the base joint.
    // */


    // visual_tools.deleteAllMarkers();

    //  Remote control is an introspection tool that allows users to step through a high level script
    //    via buttons and keyboard shortcuts in RViz 
    // visual_tools.loadRemoteControl();

    // /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    // text_pose.translation().z() = 1.75;
    // visual_tools.publishText(text_pose, "This is a test to display text", rvt::WHITE, rvt::XLARGE);

    // /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    // visual_tools.trigger();

    /* Sleep a little to allow time to startup rviz, etc..
       This ensures that visual_tools.prompt() isn't lost in a sea of logs*/
    ros::Duration(10).sleep();

    /* We can also use visual_tools to wait for user input */
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

      // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the arm of the Panda
    // specifying the desired pose of the end-effector as input.
    ROS_INFO("About to move into the Planning section");
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;

    //PoseStamped apparently uses quaternion to avoid singularities (orientation)
    pose.header.frame_id = "platform_base";
    pose.pose.position.x = 1.3306;
    pose.pose.position.y = 2;
    pose.pose.position.z = 3.64;
    pose.pose.orientation.w = 1.0;

      // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

      // We will create the request as a constraint using a helper function available
    // from the
    // `kinematic_constraints`_
    // package.
    //
    // .. _kinematic_constraints:
    //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
    req.group_name = "Arm_Group";
    req.allowed_planning_time = 3;
    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("armLink7square", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

      // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    // ros::Publisher display_publisher =
    //     node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    // moveit_msgs::DisplayTrajectory display_trajectory;

    // /* Visualize the trajectory */
    // ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    // display_trajectory.trajectory_start = response.trajectory_start;
    // display_trajectory.trajectory.push_back(response.trajectory);
    // display_publisher.publish(display_trajectory);

    // ROS_INFO_STREAM("If we see this, the file is actuallly building");
    // /* We can also use visual_tools to wait for user input */
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");




    /////////////////////////
    //SECOND PART of the Code 
    /////////////////////////


    if (!node_handle.getParam("chomp_plugin", planner_plugin_name)) 
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }

    //INSERT CODE HERE TO ADD Default Collision Detector
  //TODO: add back if you want to use CHOMP
  //  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create(), true);

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    /* First, set the state in the planning scene to the final state of the last plan */
  //  planning_scene->setCurrentState(response.trajectory_start);
  //  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  //  moveit::core::robotStateToRobotStateMsg(*robot_state,req.start_state);

    //set start state to be 0 again.
    req.allowed_planning_time = 20;

    robot_state::RobotState start_state(robot_model);
    std::vector<double> start_joint_vals = {0.0,0.0,0.0,0.0,0.0,0.0,0,0};
    start_state.setJointGroupPositions(joint_model_group, start_joint_vals);
    //TODO consider perhaps uncommenting below lines?
    planning_scene->setCurrentState(start_state); //maybe this
    robot_state->setJointGroupPositions(joint_model_group, start_joint_vals);  //maybe
    moveit::core::robotStateToRobotStateMsg(start_state,req.start_state);

    // Now, setup a joint space goal
    robot_state::RobotState goal_state(robot_model);
    moveit::core::robotStateMsgToRobotState(req.start_state,goal_state);
    goal_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);



    //Now, provide an initial seed trajectory

    // moveit_msgs::TrajectoryConstraints res;
    // == req.trajectory_constraints
    trajectory_msgs::JointTrajectory seed = response.trajectory.joint_trajectory;
    const auto dof = seed.joint_names.size();

    for (size_t i = 0; i < seed.points.size(); ++i) // for each time step
    {
      moveit_msgs::Constraints c;

      if (seed.points[i].positions.size() != dof)
        throw std::runtime_error("All trajectory position fields must have same dimensions as joint_names");

      for (size_t j = 0; j < dof; ++j) // for each joint
      {
        moveit_msgs::JointConstraint jc;
        jc.joint_name = seed.joint_names[j];
        jc.position = seed.points[i].positions[j];

        c.joint_constraints.push_back(jc);
      }
      req.trajectory_constraints.constraints.push_back(std::move(c));
    }

   // robot_state::RobotState goal_state(robot_model);
   // std::vector<double> joint_values = {0.0 , 0.6 , 0.5 , 0.6 , 0.0 , 0.0 , 0.0 };
   // goal_state.setJointGroupPositions(joint_model_group, joint_values);
   // moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
   // req.goal_constraints.clear();
   // req.goal_constraints.push_back(joint_goal);

    // Call the planner and visualize the trajectory
    /* Re-construct the planning context */
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    /* Call the Planner */
    context->solve(res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }
    /* Visualize the trajectory */

    //Following code to separate STOMP from FMT*
    // ros::Publisher display_publisher_stomp =
    //     node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path_stomp", 1, true);
    // moveit_msgs::DisplayTrajectory display_trajectory_stomp;

    // /* Visualize the trajectory */
    // ROS_INFO("Visualizing the trajectory");
    // res.getMessage(response);
    // display_trajectory_stomp.trajectory_start = response.trajectory_start;
    // display_trajectory_stomp.trajectory.push_back(response.trajectory);

    // display_publisher_stomp.publish(display_trajectory);

    // // ROS_INFO("Visualizing the trajectory");
    // // res.getMessage(response);
    // // display_trajectory.trajectory_start = response.trajectory_start;
    // // display_trajectory.trajectory.push_back(response.trajectory);

    // // /* Now you should see two planned trajectories in series*/
    // // display_publisher.publish(display_trajectory);

    // /* Wait for user input */
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  }
  



}
