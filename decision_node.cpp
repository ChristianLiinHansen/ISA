#include "ros/ros.h"
#include <isa_project/r_and_theta.h>   // Added to include my custum msg file, r_and_theta.msg
#include <geometry_msgs/Twist.h>       // Added to include the in-built msg file, geometry_msg/Twist.msg

#include <isa_project/num.h>

using namespace std;

// Using the real camera attatch to the robot, the dimension is:
#define imgCols 640
#define imgRows 480

// The r_and_theta callback function
void r_and_thetaCallBack(const isa_project::r_and_theta::ConstPtr& msg)
{
	double r = msg-> r;
	double theta = msg-> theta;
	
    //std::cout << "r: " << r << std::endl;
    //std::cout << "theta: " << theta << std::endl;
	
	geometry_msgs::Twist twist;
	ros::NodeHandle n;
	
	// Publisher cmd_velo
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
    // Perfect heading forward is 90 degree

    // Now we need to look into the r. This has the most influence on the direction.

    // If we meassure 10 degree less than 90 degree, we must stear more to the right.


    // Note:
    /*

      Normally if the vertical line is place directly in the middle of the image, the
      parameters is:

        r = 320
        theta = 90


    */

    cout << "r is: " << r << endl;
    // if r is more visualized in the left side and still is almost vertical.
    if ((r <= (imgCols*0.5-50)) && ((theta >= 80) && (theta <= 100)))
    {
        cout << "line is in left side and vertical" << endl;
        twist.angular.z = 0.1;
        twist.linear.x = 0.1;
        cout << "twist.angular.z = 0.1;" << endl;
        cout << "twist.linear.z = 0.1;" << endl;
        cout << "\n" << endl;
        cmd_vel_pub.publish(twist);
    }
    // else if r is more visualized in the right side and still is almost vertical.
    else if ((r >= (imgCols*0.5+50)) && ((theta >= 80) && (theta <= 100)))
    {
        cout << "line is in right side and vertical" << endl;
        twist.angular.z = -0.1;
        twist.linear.x = 0.1;
        cout << "twist.angular.z = -0.1;" << endl;
        cout << "twist.linear.z = 0.1;" << endl;
        cout << "\n" << endl;
        cmd_vel_pub.publish(twist);
    }

    // else r is in between 1/3 and 2/3 of the colums and we can check on the angle only
    else
    {
        if ((r <= (imgCols*0.5-50)) && (theta <= 85))
        {
            cout << "line is in the left side and tilted to the right" << endl;

            if((theta <= 85) && (theta >= 70))
            {
                twist.angular.z = -0.1;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = -0.1;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else if ((theta < 70) && (theta >= 60))
            {
                twist.angular.z = -0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = -0.2;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else
            {
                twist.angular.z = -0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = -0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
        }
        else if ((r <= (imgCols*0.5-50)) && (theta >= 95))
        {
            cout << "line is in the left side and tilted to the left" << endl;

            if((theta >= 100) && (theta <= 110))
            {
                twist.angular.z = 0.1;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = 0.1;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else if ((theta > 110) && (theta <= 120))
            {
                twist.angular.z = 0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = 0.;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else
            {
                twist.angular.z = 0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = 0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
        }
        else if ((r >= (imgCols*0.5+50)) && (theta <= 85))
        {
            cout << "line is in the right side and tilted to the right" << endl;
            cout << "\n" << endl;

            if((theta <= 85) && (theta >= 70))
            {
                twist.angular.z = -0.1;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = -0.1;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else if ((theta < 70) && (theta >= 60))
            {
                twist.angular.z = -0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = -0.2;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else
            {
                twist.angular.z = -0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = -0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
        }
        else if ((r >= (imgCols*0.5+50)) && (theta >= 95))
        {
            cout << "line is in the right side and tilted to the left" << endl;
            cout << "\n" << endl;

            if((theta >= 100) && (theta <= 110))
            {
                twist.angular.z = 0.1;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = 0.1;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else if ((theta > 110) && (theta <= 120))
            {
                twist.angular.z = 0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = 0.2;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
            else
            {
                twist.angular.z = 0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = 0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
                cmd_vel_pub.publish(twist);
            }
        }
        else
        {
            cout << "All perfect ..." << endl;
            twist.angular.z = 0.0;
            twist.linear.x = 0.1;
            cout << "twist.angular.z = 0.0;" << endl;
            cout << "twist.linear.z = 0.1;" << endl;
            cout << "\n" << endl;
            cmd_vel_pub.publish(twist);
        }
    }
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
