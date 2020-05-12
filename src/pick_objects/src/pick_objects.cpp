#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const double PICK_POS_X = 2.0;
const double PICK_POS_Y = 3.0;
const double PICK_ORI_W = 0.3;

const double DROP_OFF_POS_X =  2.0;
const double DROP_OFF_POS_Y =  2.0;
const double DROP_OFF_ORI_W =  0.4;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objets");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Duration(5).sleep(); // sleep for 5sec

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x    = PICK_POS_X;
  goal.target_pose.pose.position.y    = PICK_POS_Y;
  goal.target_pose.pose.orientation.w = PICK_ORI_W;

   // Send the pickup position and orientation for the robot to reach
  ROS_INFO("Sending pickup position.");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Success to pick the object!");
  else{
    ROS_INFO("Fail to pick the object.");
    return 0;
  }

  ros::Duration(5).sleep(); // sleep for 5sec

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x    = DROP_OFF_POS_X;
  goal.target_pose.pose.position.y    = DROP_OFF_POS_Y;
  goal.target_pose.pose.orientation.w = DROP_OFF_ORI_W;

   // Send the drop off position and orientation for the robot to reach
  ROS_INFO("Sending drop off position.");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Success to drop off the object!");
  else
    ROS_INFO("Fail to drop off the object.");

  ros::Duration(5).sleep(); // sleep for 5sec

  return 0;
}