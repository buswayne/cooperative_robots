#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


double x_c2 = 0;
double y_c2 = 0;
double x_c3 = 0;
double y_c3 = 0;

void clover2_pose_callback( const geometry_msgs::PoseWithCovarianceStamped& clover2_pose){
  x_c2 = clover2_pose.pose.pose.position.x;
  y_c2 = clover2_pose.pose.pose.position.y;
}

void clover3_pose_callback( const geometry_msgs::PoseWithCovarianceStamped& clover3_pose){
  x_c3 = clover3_pose.pose.pose.position.x;
  y_c3 = clover3_pose.pose.pose.position.y;
}

int main(int argc, char** argv){
  double x_goal = 0;
  double y_goal = 0;

  ros::init(argc, argv, "aggregation");

  ros::NodeHandle nh;
  ros::Subscriber clover2_pose = nh.subscribe("clover2/amcl_pose", 10000, clover2_pose_callback);
  ros::Subscriber clover3_pose = nh.subscribe("clover3/amcl_pose", 10000, clover3_pose_callback);
  ros::Publisher clover2_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("clover2/move_base_simple/goal", 50);
  ros::Publisher clover3_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("clover3/move_base_simple/goal", 50);


  ros::Rate r(0.66);

  while(nh.ok()){

    // subscribers
    ros::spinOnce();


    x_goal = x_c2 - (x_c2 - x_c3)/2;
    y_goal = y_c2 - (y_c2 - y_c3)/2;



    // publishers

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = x_goal;
    goal.pose.position.y = y_goal;
    goal.pose.position.z = 0;

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    //publish the message
    clover2_goal_pub.publish(goal);
    clover3_goal_pub.publish(goal);


    r.sleep();
  }
}
