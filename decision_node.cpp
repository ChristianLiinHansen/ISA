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
	
    std::cout << "r: " << r << std::endl;
    std::cout << "theta: " << theta << std::endl;
	
	geometry_msgs::Twist twist;
	ros::NodeHandle n;
	
	// Publisher cmd_velo
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
    // Perfect heading forward is 90 degree

    // If we meassure 10 degree less than 90 degree, we must stear more to the right.
    if (theta <= 80)
    {
        if((theta <= 80) && (theta >= 70))
        {
            twist.angular.z = -0.1;
            twist.linear.x = 0.1;
        }
        else if ((theta < 70) && (theta >= 60))
        {
            twist.angular.z = -0.2;
            twist.linear.x = 0.1;
        }
        else
        {
            twist.angular.z = -0.3;
            twist.linear.x = 0.0;
        }

    }
    else if(theta >= 100)
    {
        if((theta >= 100) && (theta <= 110))
        {
            twist.angular.z = 0.1;
            twist.linear.x = 0.1;
        }
        else if ((theta > 110) && (theta <= 120))
        {
            twist.angular.z = 0.2;
            twist.linear.x = 0.1;
        }
        else
        {
            twist.angular.z = 0.3;
            twist.linear.x = 0.0;
        }
    }
    else
    {
        twist.angular.z = 0.0;
        twist.linear.x = 0.1;
    }
	cmd_vel_pub.publish(twist);
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
	
    // Publisher cmd_vel
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	while (ros::ok())
	{
		// Publish the r and theta trough ROS
//		twist.linear.x = 0.2;
//		twist.linear.y = 0.0;
//		twist.linear.z = 0.0;
//		
//		twist.angular.x = 0.0;
//		twist.angular.y = 0.0;
//		twist.angular.z = 0.0;
		
		//cout << "Do we come to this point?" << endl;
		//msg.number = 23;
		//test_pub.publish(msg);
		
		// And then we send it on the test_pub topic
		//cmd_vel_pub.publish(twist);
		
        //loop_rate.sleep();
		ros::spinOnce();
	}	
	return 0;
}
