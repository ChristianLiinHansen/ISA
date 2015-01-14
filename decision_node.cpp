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
    private:
    ///////////////////////////////////////////////////////////////////
    // Private variables
    ///////////////////////////////////////////////////////////////////

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    // Declare the subscribers
    ros::Subscriber vision_sub;
    ros::Subscriber orientation_sub;

    ///////////////////////////////////////////////////////////////////
    // Private member functions
    ///////////////////////////////////////////////////////////////////

    public:

        ///////////////////////////////////////////////////////////////////
        // Public variables
        ///////////////////////////////////////////////////////////////////

        double r, theta, yaw;
        bool end_of_line;

        // Decklare the message
        geometry_msgs::Twist twist;  // Works for simulation
        geometry_msgs::TwistStamped twistStamped; // Testing for real running

        // Declare the publishers
        ros::Publisher cmd_vel_pub;
        ros::Publisher cmd_vel_vision_pub;

        ///////////////////////////////////////////////////////////////////
        // Public member functions
        ///////////////////////////////////////////////////////////////////

        DecisionClass()
        {
            // Initialize the subscribers in the constructor
            vision_sub = n.subscribe("/vision", 1000, &DecisionClass::vision_CallBack, this);
            orientation_sub = n.subscribe("/orientation", 1000, &DecisionClass::orientation_CallBack, this);

            /**
           * The advertise() function is how you tell ROS that you want to
           * publish on a given topic name. This invokes a call to the ROS
           * master node, which keeps a registry of who is publishing and who
           * is subscribing. After this advertise() call is made, the master
           * node will notify anyone who is trying to subscribe to this topic name,
           * and they will in turn negotiate a peer-to-peer connection with this
           * node.  advertise() returns a Publisher object which allows you to
           * publish messages on that topic through a call to publish().  Once
           * all copies of the returned Publisher object are destroyed, the topic
           * will be automatically unadvertised.
           *
           * The second parameter to advertise() is the size of the message queue
           * used for publishing messages.  If messages are published more quickly
           * than we can send them, the number here specifies how many messages to
           * buffer up before throwing some away.
           */
            // Initialize the publishers in the constructor
            cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/fmCommand/cmd_vel_from_vision", 1);
            cmd_vel_vision_pub = n.advertise<geometry_msgs::TwistStamped>("/fmCommand/cmd_vel", 1);

        }

        ~DecisionClass()
        {

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
            r = msg-> r;
            theta = msg-> theta;
            end_of_line = msg-> end_of_line;
        }

        // The r_and_theta callback function
        void orientation_CallBack(const isa_project::orientation::ConstPtr& msg)
        {
            double yaw = msg-> yaw;
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

        void CheckForEndOfLine()
        {
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
};

int main(int argc, char **argv)
{
    //  Initialize this ROS node
    ros::init(argc, argv, "decision_node");

    // Initialize the class DecisionClass
    DecisionClass dc;

    //ros::NodeHandle n;
    // Try to set the loop rate down to avoid the watchdog timer to set linary and velocity to zero.
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // Check if the robot has in end_of_line
        dc.CheckForEndOfLine();

        // And then we send it on the cmd_vel_pub and cmd_vel_vision_pub topics
        dc.cmd_vel_pub.publish(dc.twistStamped);
        dc.cmd_vel_vision_pub.publish(dc.twistStamped);

        // And wait
        loop_rate.sleep();

        // Make sure that we enter the call back functions onces
        ros::spinOnce();
    }
    return 0;
}
