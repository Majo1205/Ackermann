#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

// custom msg (not used below)
//#include "ackermann_control/velocities.h"
//#include "ackermann_control/pose_msg.h" //EVENTUALLY USE A pose2d msg

#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

# define M_PI 3.14159265358979323846  // pi

class Ackermann
{

	private:
	// Variables
	float length, width;
	float scaling,k1,k2,k3,threshold;
  float dx,dy,rho,theta,gamma,delta;
  std_msgs::Float64 theta_goal;
  float bar_omega;
	std_msgs::Float64 v,phi;
  double actual_yaw;

	nav_msgs::Odometry robot_odom;
  ackermann_control::pose_msg desired_goal;

	//NodeHandles
	ros::NodeHandle n; // Subscribers
	ros::NodeHandle n_; // Publishers

  // Subscriber
	ros::Subscriber sub_odom;

  // Publishers
	ros::Publisher pub_left_steer;
	ros::Publisher pub_right_steer;

	ros::Publisher pub_rear_left_wheel;
	ros::Publisher pub_rear_right_wheel;
	ros::Publisher pub_front_left_wheel;
	ros::Publisher pub_front_right_wheel;


	public:
	Ackermann()
	{
	// Ackermann Robot Initialization

	length = 1; //meters
	width = 0.7;

	// control parameters
	scaling = 0.7;
	k1 = 1.5*scaling;
	k2 = 1*scaling;
	k3 = 3*scaling;


	// thereshold to stop Robot (Euclidian distance)
	threshold = 0.1;

	// initial robot pose (to 0)
	robot_odom.pose.pose.position.x = 0;
  robot_odom.pose.pose.position.y = 0;
  robot_odom.pose.pose.position.z = 0;
  robot_odom.pose.pose.orientation.x = 0;
  robot_odom.pose.pose.orientation.y = 0;
  robot_odom.pose.pose.orientation.z = 0;
  robot_odom.pose.pose.orientation.w = 0;

  ///////////// set desired goal position  //////////////
  desired_goal.x.data = 6;
  desired_goal.y.data = -4;
  desired_goal.theta.data = -30; //degrees

	// sub at /ground_truth (pose with covariance of base_link)
	sub_odom = n.subscribe("ground_truth", 1000, &Ackermann::callbackPose, this); //ground_truth (Odometry)

	// publishers steering
	pub_left_steer  = n_.advertise<std_msgs::Float64>("/ackermann/front_left_steering_controller/command", 1000);
  pub_right_steer = n_.advertise<std_msgs::Float64>("/ackermann/front_right_steering_controller/command", 1000);
	// publishers velocities
	pub_rear_left_wheel   = n_.advertise<std_msgs::Float64>("/ackermann/rear_left_wheel_controller/command", 1000);
	pub_rear_right_wheel  = n_.advertise<std_msgs::Float64>("/ackermann/rear_right_wheel_controller/command", 1000);
	pub_front_left_wheel  = n_.advertise<std_msgs::Float64>("/ackermann/front_left_wheel_controller/command", 1000);
	pub_front_right_wheel = n_.advertise<std_msgs::Float64>("/ackermann/front_right_wheel_controller/command", 1000);

	}

	//callback when ground_truth is published
	void callbackPose(const nav_msgs::Odometry msg) //the callback rate is set by ground_truth plugin
	{
    robot_odom = msg;

  	if (distance(robot_odom,desired_goal) > threshold)
   	{
     	ROS_INFO("Approaching goal");
     	ROS_INFO("Euclidian Distance:[%f]", distance(robot_odom,desired_goal));
     	geometry_msgs::Vector3 RollPitchYaw;

     	// starting control
     	dx = desired_goal.x.data-robot_odom.pose.pose.position.x;
     	dy = desired_goal.y.data-robot_odom.pose.pose.position.y;
     	rho = sqrt(pow(dx,2)+pow(dy,2)); //distance

     	RollPitchYaw = quat2RPY(robot_odom.pose.pose.orientation);
     	theta = RollPitchYaw.z;

     	gamma = atan2(dy, dx) - theta; //first angle
     	theta_goal = deg2rad(desired_goal.theta);
     	delta = gamma + theta - theta_goal.data; //second angle

     	//////// CONTROL ////////
     	// linear velocity
     	v.data = k1*rho*(sign_(cos(gamma)));
     	//angular velocity
     	bar_omega = k2*gamma + k1*(sin(gamma)*(sign_(cos(gamma)))/gamma)*(gamma+ k3*delta);
     	//corresponding angle control phi
     	bar_omega = bar_omega*length/v.data;
     	phi.data = sign_(bar_omega)*acos(1/sqrt(pow(bar_omega,2)+1));

			//send control
     	sendVelocities(v,phi);

   	}

   	else
   	{
     	ROS_INFO("On the goal. Stopping motors");
		 	std_msgs::Float64 stop_v;
		 	std_msgs::Float64 stop_phi;
		 	stop_v.data = 0;
		 	stop_phi.data = 0;

			//send 0 velocity
		 	sendVelocities(stop_v,stop_phi);
   	}


  }

  float distance(const nav_msgs::Odometry actual_pos, ackermann_control::pose_msg goal)
  {
    geometry_msgs::Vector3 RPY_angles;
    float goal_distance;

    RPY_angles = quat2RPY(actual_pos.pose.pose.orientation);

    goal_distance = sqrt(pow((goal.x.data-actual_pos.pose.pose.position.x),2)+pow((goal.y.data-actual_pos.pose.pose.position.y),2));//+pow((goal.theta.data-RPY_angles.z),2);

    return goal_distance;
  }

	void sendVelocities(const std_msgs::Float64 vel, std_msgs::Float64 steer)
	{
		// steering
		pub_left_steer.publish(steer);
		pub_right_steer.publish(steer);

		// wheel velocity
		pub_rear_left_wheel.publish(vel);
		pub_rear_right_wheel.publish(vel);
		pub_front_left_wheel.publish(vel);
		pub_front_right_wheel.publish(vel);

	}

  std_msgs::Float64 deg2rad(std_msgs::Float64 angle_degrees)
  {
    // std_msgs::Float64 angle_radians;
    std_msgs::Float64 angle_radians;
    angle_radians.data = (M_PI/180) * angle_degrees.data;

    return angle_radians;
  }

  geometry_msgs::Vector3 quat2RPY(geometry_msgs::Quaternion quat_msg)
  {
    geometry_msgs::Vector3 RollPitchYaw;
    double roll, pitch, yaw;

    tf::Quaternion q(quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    RollPitchYaw.x = roll;
    RollPitchYaw.y = pitch;
    RollPitchYaw.z = yaw;

    return RollPitchYaw;

  }

  //different definition for sign
  float sign_(float num)
  {
    float res = 1;

    if (num<0)
    res = -1;

    return res;
  }

}; //end class Ackermann



int main(int argc, char **argv)
{

	ros::init(argc, argv, "controller_node");


	Ackermann ClassObj;


	ros::spin();

	return 0;

}
