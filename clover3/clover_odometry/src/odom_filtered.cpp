#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;

void poseCallback( const geometry_msgs::PoseWithCovarianceStamped& pose_ekf){
  x = pose_ekf.pose.pose.position.x;
  y = pose_ekf.pose.pose.position.y;
  th = pose_ekf.pose.pose.orientation.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "filtered_odometry_publisher");

  ros::NodeHandle nh;
  ros::Subscriber pose_ekf_sub = nh.subscribe("robot_pose_ekf/odom_combined", 10000, poseCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("filtered_odom", 500);

  double x_prev = 0.0;
  double th_prev = 0.0;
  double v_x = 0.0;
  double v_y = 0.0;
  double v_th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50);
  while(nh.ok()){

    ros::spinOnce();
    current_time = ros::Time::now();
    //compute actual velocities of the robot
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    double delta_x = x - x_prev;
    double delta_th = th - th_prev;

    
    v_x = delta_x /(cos(th)*dt);
    v_th = delta_th /dt;


    //since all odometry is 6DOF we'll need a quaternion created from yaw(rotation along z)
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);


    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry filtered_odom;
    filtered_odom.header.stamp = current_time;
    filtered_odom.header.frame_id = "odom";

    //set the position
    filtered_odom.pose.pose.position.x = x;
    filtered_odom.pose.pose.position.y = y;
    filtered_odom.pose.pose.position.z = 0.0;
    filtered_odom.pose.pose.orientation = odom_quat;

    //set the velocity
    filtered_odom.child_frame_id = "robot_footprint";
    filtered_odom.twist.twist.linear.x = v_x;
    filtered_odom.twist.twist.linear.y = v_y;
    filtered_odom.twist.twist.angular.z = v_th;



    //publish the message
    odom_pub.publish(filtered_odom);

    last_time = current_time;
    x_prev = x;
    th_prev = th;
    r.sleep();
  }

}

