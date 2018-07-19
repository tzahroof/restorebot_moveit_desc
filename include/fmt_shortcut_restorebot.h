#include <math.h>
#include <moveit_msgs/PlanningScene.h>

static double determineCost(moveit_msgs::MotionPlanResponse &response);
static void visualizePlot(moveit_msgs::MotionPlanResponse &response);