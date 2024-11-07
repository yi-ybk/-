#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>

/*******全局变量定义区********/
/*初始化*/
double target_A_x = 0.55;
double target_A_y = -0.49;
double target_B_x = 0.55;
double target_B_y = -1.64;
double current_x = 0;
double current_y = 0;
double current_yaw = 0;
double target_yaw = -M_PI/2;
double distance_to_stop = 0.3;
double front_distance;
bool goal_reached_A = false;
bool goal_reached_B = false;
bool goal_yaw_A = false;
geometry_msgs::Twist twist;
/***************************/

/******函数声明区*******/
void scancallback(const sensor_msgs::LaserScan::ConstPtr& msg);
double quat_to_yaw(const geometry_msgs::Quaternion& quat);
void odomcallback(const nav_msgs::Odometry& msg);
/*********************/

int main(int argc, char **argv){
  ros::init(argc, argv, "move_node");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber odom_sub = n.subscribe("/odom", 10, odomcallback);
  ros::Subscriber scan_sub = n.subscribe("scan", 10, scancallback);
  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    pub.publish(twist);
  }
  return 0;
}

double quat_to_yaw(const geometry_msgs::Quaternion& quat) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void odomcallback(const nav_msgs::Odometry& msg){
  current_x = msg.pose.pose.position.x;
  current_y = msg.pose.pose.position.y;
  /*将四元数转化为角度*/
  current_yaw = quat_to_yaw(msg.pose.pose.orientation);
  /*计算角度差*/
  double yaw_diff = std::fabs(current_yaw - target_yaw);

  /*向A点出发*/
  if(goal_reached_A == false && current_x < target_A_x){
    twist.linear.x = 0.2;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    /*检测是否到达A点*/
    if(current_x >= target_A_x){
      goal_reached_A = true;
    }
  }

  /*转向*/
  else if(goal_yaw_A == false && yaw_diff > 0.1){
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = -0.5;
    if(yaw_diff <= 0.1){
      goal_yaw_A = true;
    }
  }
}

void scancallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  front_distance = msg->ranges[0];
  ROS_INFO("距离 = %lf", front_distance);

  if(front_distance <= distance_to_stop ){
      goal_reached_B = true;
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.linear.z = 0;
      twist.angular.x = 0;
      twist.angular.y = 0;
      twist.angular.z = 0;
  }

  /*剩余0.3m时停止*/
  else if(goal_reached_B == false){
    twist.linear.x = 0.2;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
  }
}


