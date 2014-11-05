#include "ros/ros.h"
#include <isa_project/r_and_theta.h>   // Added to include my custum msg file, r_and_theta.msg
#include <geometry_msgs/Twist.h>       // Added to include the in-built msg file, geometry_msg/Twist.msg

#include <isa_project/num.h>

using namespace std;

// The r_and_theta callback function
void r_and_thetaCallBack(const isa_project::r_and_theta::ConstPtr& msg)
{
	double r = msg-> r;
	double theta = msg-> theta;
	
	std::cout << "I heard this float64: " << r << std::endl;
	std::cout << "I heard this float64: " << theta << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "decision_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	geometry_msgs::Twist twist;
	isa_project::num msg;
	
	// Subscriber r_and_theta
	ros::Subscriber r_and_theta_sub = n.subscribe("/r_and_theta", 1000, r_and_thetaCallBack);
	
	// Publisher cmd_velo
	ros::Publisher cmd_velo_pub = n.advertise<geometry_msgs::Twist>("/cmd_velo", 1);
	ros::Publisher test_pub = n.advertise<isa_project::num>("/testing_num", 1);
	
	while (ros::ok())
	{
		// Publish the r and theta trough ROS
		twist.linear.x = 1.0;
		twist.linear.y = 2.0;
		twist.linear.z = 3.0;
		
		twist.angular.x = 0.1;
		twist.angular.y = 0.2;
		twist.angular.z = 0.3;
		
		cout << "Do we come to this point?" << endl;
		msg.number = 23;
		test_pub.publish(msg);
		
		// And then we send it on the test_pub topic
		cmd_velo_pub.publish(twist);
		
		loop_rate.sleep();
		ros::spinOnce();
	}	
	return 0;
}
