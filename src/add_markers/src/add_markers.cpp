#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

const double PICK_POS_X = 2.0;
const double PICK_POS_Y = 3.0;
const double PICK_ORI_W = 0.3;

const double DROP_OFF_POS_X =  2.0;
const double DROP_OFF_POS_Y =  2.0;
const double DROP_OFF_ORI_W =  0.4;

const double threshold   = 0.2;
const double threshold_w = 0.7;

bool pickup_flag = false;

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

    //pose check 
    double errX = PICK_POS_X - msg->pose.pose.position.x;
    double errY = PICK_POS_Y - msg->pose.pose.position.y;
    double errW = PICK_ORI_W - msg->pose.pose.orientation.w;

    double pickup_error   = sqrt( errX * errX + errY * errY );
    double pickup_error_w = sqrt( errW * errW );

    errX = DROP_OFF_POS_X - msg->pose.pose.position.x;
    errY = DROP_OFF_POS_Y - msg->pose.pose.position.y;
    errW = DROP_OFF_ORI_W - msg->pose.pose.orientation.w;

    double dropoff_error   = sqrt( errX * errX + errY * errY);
    double dropoff_error_w = sqrt( errW * errW); 

    if(!pickup_flag && pickup_error < threshold && pickup_error_w < threshold_w){
        pickup_flag = true;
        ROS_INFO("Picked up. Delete Marker");
        ROS_INFO("X: %f", msg->pose.pose.position.x);
        ROS_INFO("Y: %f", msg->pose.pose.position.y);
        ROS_INFO("W: %f", msg->pose.pose.orientation.w);

        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

        ros::Duration(5).sleep(); // sleep for 5sec
    }

    if( pickup_flag && dropoff_error < threshold && dropoff_error_w < threshold_w){
        pickup_flag = false;

        marker.pose.position.x    = DROP_OFF_POS_X;
        marker.pose.position.y    = DROP_OFF_POS_Y;
        marker.pose.orientation.w = DROP_OFF_ORI_W;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);

        ROS_INFO("Drop off. Put Marker");
        ROS_INFO("X: %f", msg->pose.pose.position.x);
        ROS_INFO("Y: %f", msg->pose.pose.position.y);
        ROS_INFO("W: %f", msg->pose.pose.orientation.w);
    }

}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Subscriber amclPose_sub = n.subscribe("amcl_pose", 100, amclPoseCallback);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = PICK_POS_X;
    marker.pose.position.y = PICK_POS_Y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = PICK_ORI_W;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
        return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    marker_pub.publish(marker);

    ros::spin();

    return 0;

}
