#include "ros/ros.h"
#include <isa_project/vision.h>                 // Added to include my custom msg file, vision.msg
#include <isa_project/orientation.h>            // Added to include my custom msg file, orientation.msg
#include <geometry_msgs/Twist.h>                // Added to include the in-built msg file, geometry_msg/Twist.msg
#include <geometry_msgs/TwistStamped.h>         // Added to include the in-built msg file, geometry_msg/TwistStamped.msg

using namespace std;                   // Added to avoid typing std::function all over the place, like std::cout and std::endl

// Using the simulated input image
//#define imgCols 1088
//#define imgRows 612

// Using the real camera attach to the robot, the dimension is:
#define imgCols 640
#define imgRows 480

#define linear_stopped 0.0
#define angular_stopped 0.0

#define linear_vel 0.2

#define angular_vel1 0.3
#define angular_vel2 0.4
#define angular_vel3 0.5

class DecisionClass
{
    public:

        ///////////////////////////////////////////////////////////////////
        // Public variables
        ///////////////////////////////////////////////////////////////////

        double yaw, degree;

        ///////////////////////////////////////////////////////////////////
        // Public member functions
        ///////////////////////////////////////////////////////////////////

        DecisionClass()
        {

        }

        ~DecisionClass()
        {
            //cout << "Calling the deconstructor" << endl;
        }

    private:

        ///////////////////////////////////////////////////////////////////
        // Private variables
        ///////////////////////////////////////////////////////////////////


        ///////////////////////////////////////////////////////////////////
        // Private member functions
        ///////////////////////////////////////////////////////////////////
};

// Global varialbe
geometry_msgs::Twist twist;  // Works for simulation
geometry_msgs::TwistStamped twistStamped; // Testing for real running

// Functions
void GetOrientation()
{

}

void GetTwist(double r, double theta)
{
    cout << "r is: " << r << endl;
    // if r is more visualized in the left side and still is almost vertical.
    if ((r <= (imgCols*0.5-50)) && ((theta >= 80) && (theta <= 100)))
    {
        cout << "line is in left side and vertical" << endl;
        twistStamped.twist.angular.z = angular_vel1;
        twistStamped.twist.linear.x = linear_vel;
        cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
        cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
        cout << "\n" << endl;
    }
    // else if r is more visualized in the right side and still is almost vertical.
    else if ((r >= (imgCols*0.5+50)) && ((theta >= 80) && (theta <= 100)))
    {
        cout << "line is in right side and vertical" << endl;
        twistStamped.twist.angular.z = -angular_vel1;
        twistStamped.twist.linear.x = linear_vel;
        cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
        cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
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
                twistStamped.twist.angular.z = -angular_vel1;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else if ((theta < 70) && (theta >= 60))
            {
                twistStamped.twist.angular.z = -angular_vel2;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else
            {
                twistStamped.twist.angular.z = -angular_vel3;
                twistStamped.twist.linear.x = linear_stopped;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
        }
        else if ((r <= (imgCols*0.5-50)) && (theta >= 95))
        {
            cout << "line is in the left side and tilted to the left" << endl;

            if((theta >= 100) && (theta <= 110))
            {
                twistStamped.twist.angular.z = angular_vel1;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else if ((theta > 110) && (theta <= 120))
            {
                twistStamped.twist.angular.z = angular_vel2;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else
            {
                twistStamped.twist.angular.z = angular_vel3;
                twistStamped.twist.linear.x = linear_stopped;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
        }
        else if ((r >= (imgCols*0.5+50)) && (theta <= 85))
        {
            cout << "line is in the right side and tilted to the right" << endl;
            cout << "\n" << endl;

            if((theta <= 85) && (theta >= 70))
            {
                twistStamped.twist.angular.z = -angular_vel1;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else if ((theta < 70) && (theta >= 60))
            {
                twistStamped.twist.angular.z = -angular_vel2;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else
            {
                twistStamped.twist.angular.z = -angular_vel3;
                twistStamped.twist.linear.x = linear_stopped;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
        }
        else if ((r >= (imgCols*0.5+50)) && (theta >= 95))
        {
            cout << "line is in the right side and tilted to the left" << endl;
            cout << "\n" << endl;

            if((theta >= 100) && (theta <= 110))
            {
                twistStamped.twist.angular.z = angular_vel1;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else if ((theta > 110) && (theta <= 120))
            {
                twistStamped.twist.angular.z = angular_vel2;
                twistStamped.twist.linear.x = linear_vel;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
            else
            {
                twistStamped.twist.angular.z = angular_vel3;
                twistStamped.twist.linear.x = linear_stopped;
                cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                cout << "\n" << endl;
            }
        }
        else
        {
            cout << "All perfect ..." << endl;
            twistStamped.twist.angular.z = angular_stopped;
            twistStamped.twist.linear.x = linear_vel;
            cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
            cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
            cout << "\n" << endl;
        }
    }
}

// The vision callback function
void vision_CallBack(const isa_project::vision::ConstPtr& msg)
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
    bool end_of_line = msg-> end_of_line;

    if (end_of_line == false)
    {
        cout << "End of line is false" << endl;
        GetTwist(r, theta);
    }
    else // Then we are at a end_of_line. And we should start turning 180 degree
    {
        cout << "We are at end of line and should stop" << endl;
        twistStamped.twist.angular.z = angular_stopped;
        twistStamped.twist.linear.x = linear_stopped;

    }

}

// The r_and_theta callback function
void orientation_CallBack(const isa_project::orientation::ConstPtr& msg)
{
    double yaw = msg-> yaw;
    cout << "The yaw angle trough topic is: " << yaw << endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle n;
    // Try to set the loop rate down to avoid the watchdog timer to set linary and velocity to zero.
    ros::Rate loop_rate(30);

    // Subscriber r_and_theta
    ros::Subscriber vision_sub = n.subscribe("/vision", 1000, vision_CallBack);
    ros::Subscriber orientation_sub = n.subscribe("/orientation", 1000, orientation_CallBack);

    // Publisher cmd_vel
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/fmCommand/cmd_vel_from_vision", 1);
    ros::Publisher cmd_vel_vision_pub = n.advertise<geometry_msgs::TwistStamped>("/fmCommand/cmd_vel", 1);

    while (ros::ok())
    {
        // And then we send it on the test_pub topic
        cmd_vel_pub.publish(twistStamped);
        cmd_vel_vision_pub.publish(twistStamped);

        // And wait
        loop_rate.sleep();

        // Make sure that we enter the call back functions onces
        ros::spinOnce();
    }
    return 0;

}
