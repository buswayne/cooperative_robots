#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

//define robot parameters
float R = 0.1;
float base_width = 0.25;
double left_wheel_speed = 0.0;
double right_wheel_speed = 0.0;

void left_velCallback( const std_msgs::Float32& left_speed){
  left_wheel_speed = left_speed.data;
}
void right_velCallback( const std_msgs::Float32& right_speed){
  right_wheel_speed = right_speed.data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "encoder_odometry_publisher");

  ros::NodeHandle nh;
  ros::Subscriber left_sub = nh.subscribe("left_speed", 10000, left_velCallback);
  ros::Subscriber right_sub = nh.subscribe("right_speed", 10000, right_velCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("encoder_odom", 500);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100);
  while(nh.ok()){

    ros::spinOnce();
    current_time = ros::Time::now();
    //compute actual velocities of the robot
    if(abs(left_wheel_speed - right_wheel_speed) < 0.75) {	//assume only vx, vth = 0
      vx = (left_wheel_speed + right_wheel_speed)/2*R;
      vth = 0;
      }
    else {
      vx = (left_wheel_speed + right_wheel_speed)*R/2;
      vth = (right_wheel_speed - left_wheel_speed)*R/(2*base_width);
      }
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw(rotation along z)
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped wheels_odom_trans;
    wheels_odom_trans.header.stamp = current_time;
    wheels_odom_trans.header.frame_id = "clover3/odom";
    wheels_odom_trans.child_frame_id = "clover3/robot_footprint";

    wheels_odom_trans.transform.translation.x = x;
    wheels_odom_trans.transform.translation.y = y;
    wheels_odom_trans.transform.translation.z = 0.0;
    wheels_odom_trans.transform.rotation = odom_quat;

    //send the transform
    //odom_broadcaster.sendTransform(wheels_odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry encoder_odom;
    encoder_odom.header.stamp = current_time;
    encoder_odom.header.frame_id = "clover3/odom";

    //set the position
    encoder_odom.pose.pose.position.x = x;
    encoder_odom.pose.pose.position.y = y;
    encoder_odom.pose.pose.position.z = 0.0;
    encoder_odom.pose.pose.orientation = odom_quat;

    //set the velocity
    encoder_odom.child_frame_id = "clover3/robot_footprint";
    encoder_odom.twist.twist.linear.x = vx;
    encoder_odom.twist.twist.linear.y = vy;
    encoder_odom.twist.twist.angular.z = vth;

    //set the covariances
    encoder_odom.pose.covariance = {0.03, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.03, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.03, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.3, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.3};

    encoder_odom.twist.covariance ={0.005, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.005, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.005, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.04, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.04, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.04};

    //publish the message
    odom_pub.publish(encoder_odom);

    last_time = current_time;
    r.sleep();
  }

}






