#include "ros/ros.h"
#include <isa_project/r_and_theta.h>            // Added to include my custom msg file, r_and_theta.msg
#include <geometry_msgs/Twist.h>                // Added to include the in-built msg file, geometry_msg/Twist.msg
#include <geometry_msgs/TwistStamped.h>         // Added to include the in-built msg file, geometry_msg/TwistStamped.msg

using namespace std;                   // Added to avoid typing std::function all over the place, like std::cout and std::endl

// Using the simulated input image
#define imgCols 853
#define imgRows 640

// Using the real camera attach to the robot, the dimension is:
//#define imgCols 640
//#define imgRows 480

// Global varialbe
geometry_msgs::Twist twist;  // Works for simulation
geometry_msgs::TwistStamped twistStamped; // Testing for real running

// Functions
geometry_msgs::Twist GetTwist(double r, double theta)
{
    geometry_msgs::Twist twist;
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
            }
            else if ((theta < 70) && (theta >= 60))
            {
                twist.angular.z = -0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = -0.2;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
            }
            else
            {
                twist.angular.z = -0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = -0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
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
            }
            else if ((theta > 110) && (theta <= 120))
            {
                twist.angular.z = 0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = 0.;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
            }
            else
            {
                twist.angular.z = 0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = 0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
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
            }
            else if ((theta < 70) && (theta >= 60))
            {
                twist.angular.z = -0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = -0.2;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
            }
            else
            {
                twist.angular.z = -0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = -0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
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
            }
            else if ((theta > 110) && (theta <= 120))
            {
                twist.angular.z = 0.2;
                twist.linear.x = 0.1;
                cout << "twist.angular.z = 0.2;" << endl;
                cout << "twist.linear.z = 0.1;" << endl;
                cout << "\n" << endl;
            }
            else
            {
                twist.angular.z = 0.3;
                twist.linear.x = 0.0;
                cout << "twist.angular.z = 0.3;" << endl;
                cout << "twist.linear.z = 0.0;" << endl;
                cout << "\n" << endl;
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
        }
    }

    return twist;
}

// The r_and_theta callback function
void r_and_thetaCallBack(const isa_project::r_and_theta::ConstPtr& msg)
{
    // Note:
    /*
      We get into the subscriber cb funtion each time there is a new stuff on this topic

      Normally if the vertical line is place directly in the middle of the image, the
      parameters is:

        r = 320
        theta = 90
    */
    double r = msg-> r;
    double theta = msg-> theta;

    // Overwrite to the global variable twist --> Should be optimized with classes.
    twist = GetTwist(r, theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Subscriber r_and_theta
    ros::Subscriber r_and_theta_sub = n.subscribe("/r_and_theta", 1000, r_and_thetaCallBack);

    // Publisher cmd_vel
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (ros::ok())
    {
        // And then we send it on the test_pub topic
        cmd_vel_pub.publish(twist);

        // And wait
        loop_rate.sleep();

        // Make sure that we enter the call back functions onces
        ros::spinOnce();
    }
    return 0;
}
