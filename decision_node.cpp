#include "ros/ros.h"
#include <isa_project/vision.h>                 // Added to include my custom msg file, vision.msg
#include <isa_project/orientation.h>            // Added to include my custom msg file, orientation.msg
#include <geometry_msgs/Twist.h>                // Added to include the in-built msg file, geometry_msg/Twist.msg
#include <geometry_msgs/TwistStamped.h>         // Added to include the in-built msg file, geometry_msg/TwistStamped.msg

// OpenCV lib
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <ctime>                                // For time
#include <cstdio>
#include <sstream>
using namespace std;                   // Added to avoid typing std::function all over the place, like std::cout and std::endl
using namespace cv;
#include <std_msgs/Char.h>                      // Added to subscribe on keyinputs

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
#define angular_vel3 0.8

#define follow_line_state 1
#define end_of_line_state 2
#define turning_state 3
#define start_of_line_state 4

#define runForwardSec 9
#define sleepSec 2

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
    ros::Subscriber keyboard_sub;

    ///////////////////////////////////////////////////////////////////
    // Private member functions
    ///////////////////////////////////////////////////////////////////

    public:

        ///////////////////////////////////////////////////////////////////
        // Public variables
        ///////////////////////////////////////////////////////////////////

        double r, theta, yaw;
        bool end_of_line_flag;
        unsigned char current_state;
        unsigned int keyInput;

        double beginDegree;
        double endDegree;

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
            keyboard_sub = n.subscribe("/fmHMI/keyboard", 1000, &DecisionClass::keyboard_CallBack, this);

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

        // The keyboard callback function

        void keyboard_CallBack(const std_msgs::Char::ConstPtr& msg)
        {
            cout << "We are inside the keyboard cb function" << endl;
            keyInput = msg -> data;
            cout << "KeyInput is: " << keyInput << endl;
        }

        // The vision callback function
        void vision_CallBack(const isa_project::vision::ConstPtr& msg)
        {
            //cout << "We are inside the vision cb function" << endl;
            // Note:
            /*
              We get into the subscriber cb funtion each time there is a new stuff on this topic

              Normalend_of_line_stately if the vertical line is place directly in the middle of the image, the
              parameters is:

                r = 320
                theta = 90
            */
            r = msg-> r;
            theta = msg-> theta;
            end_of_line_flag = msg-> end_of_line_flag;
        }

        // The orientation callback function
        void orientation_CallBack(const isa_project::orientation::ConstPtr& msg)
        {
            //cout << "We are inside the orientation cb function" << endl;
            yaw = msg-> yaw;
        }

        void GetTwist(double r, double theta)
        {
            //cout << "r is: "        << r << endl;
            //cout << "theta is: "    << theta << endl;

            // if r is more visualized in the left side and still is almost vertical.
            if ((r <= (imgCols*0.5-50)) && ((theta >= 80) && (theta <= 100)))
            {
                cout << "line is in left side and vertical" << endl;
                twistStamped.twist.angular.z = angular_vel1;
                twistStamped.twist.linear.x = linear_vel;

                //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                //cout << "\n" << endl;
            }
            // else if r is more visualized in the right side and still is almost vertical.
            else if ((r >= (imgCols*0.5+50)) && ((theta >= 80) && (theta <= 100)))
            {
                cout << "line is in right side and vertical" << endl;
                twistStamped.twist.angular.z = -angular_vel1;
                twistStamped.twist.linear.x = linear_vel;
                //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                //cout << "\n" << endl;
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
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else if ((theta < 70) && (theta >= 60))
                    {
                        twistStamped.twist.angular.z = -angular_vel2;
                        twistStamped.twist.linear.x = linear_vel;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else
                    {
                        twistStamped.twist.angular.z = -angular_vel3;
                        twistStamped.twist.linear.x = linear_stopped;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                }
                else if ((r <= (imgCols*0.5-50)) && (theta >= 95))
                {
                    cout << "line is in the left side and tilted to the left" << endl;

                    if((theta >= 100) && (theta <= 110))
                    {
                        twistStamped.twist.angular.z = angular_vel1;
                        twistStamped.twist.linear.x = linear_vel;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else if ((theta > 110) && (theta <= 120))
                    {
                        twistStamped.twist.angular.z = angular_vel2;
                        twistStamped.twist.linear.x = linear_vel;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else
                    {
                        twistStamped.twist.angular.z = angular_vel3;
                        twistStamped.twist.linear.x = linear_stopped;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                }
                else if ((r >= (imgCols*0.5+50)) && (theta <= 85))
                {
                    cout << "line is in the right side and tilted to the right" << endl;

                    if((theta <= 85) && (theta >= 70))
                    {
                        twistStamped.twist.angular.z = -angular_vel1;
                        twistStamped.twist.linear.x = linear_vel;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else if ((theta < 70) && (theta >= 60))
                    {
                        twistStamped.twist.angular.z = -angular_vel2;
                        twistStamped.twist.linear.x = linear_vel;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else
                    {
                        twistStamped.twist.angular.z = -angular_vel3;
                        twistStamped.twist.linear.x = linear_stopped;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                }
                else if ((r >= (imgCols*0.5+50)) && (theta >= 95))
                {
                    cout << "line is in the right side and tilted to the left" << endl;

                    if((theta >= 100) && (theta <= 110))
                    {
                        twistStamped.twist.angular.z = angular_vel1;
                        twistStamped.twist.linear.x = linear_vel;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else if ((theta > 110) && (theta <= 120))
                    {
                        twistStamped.twist.angular.z = angular_vel2;
                        twistStamped.twist.linear.x = linear_vel;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                    else
                    {
                        twistStamped.twist.angular.z = angular_vel3;
                        twistStamped.twist.linear.x = linear_stopped;
                        //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                        //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                        //cout << "\n" << endl;
                    }
                }
                else
                {
                    cout << "All perfect ..." << endl;
                    twistStamped.twist.angular.z = angular_stopped;
                    twistStamped.twist.linear.x = linear_vel;
                    //cout << "twistStamped.twist.angular.z = " << twistStamped.twist.angular.z << endl;
                    //cout << "twistStamped.twist.linear.x = " << twistStamped.twist.linear.x << endl;
                    //cout << "\n" << endl;
                }
            }
        }

        // The very simple state machine. State 1 = follow_line, state 2 = end_of_line, state 3 = turning_180, state 4 = start_of_line
        unsigned char CheckForEndOfLine()
        {
            // State 1
            if (end_of_line_flag == false)
            {
                //cout << "The end_of_line_flag = false" << endl;

                return follow_line_state;
            }
            else // Then we are at a end_of_line. And we should start turning 180 degree
            {
                // Since we entered the end_of_line, we return 1 and goes to
                // the end_of_line_state.
                //cout << "The end_of_line_flag = true" << endl;
                return end_of_line_state;
            }
        }

        unsigned char WaitForTurning(double sec)
        {
            double beginTime = ros::Time::now().toSec();
            double endTime = ros::Time::now().toSec();

            if((endTime - beginTime) <= sec)
            {
                endTime = ros::Time::now().toSec();
                return turning_state;
            }
            else
            {
                return end_of_line_state;
            }
        }

        void PublishTwist()
        {
            cmd_vel_pub.publish(twistStamped);
            cmd_vel_vision_pub.publish(twistStamped);
        }

        void StopRobot()
        {
            twistStamped.twist.angular.z = angular_stopped;
            twistStamped.twist.linear.x = linear_stopped;
            PublishTwist();
        }

        // Drive a little forward manually.
        void DriveRobotForward(double sec)
        {
            double beginTime = ros::Time::now().toSec();
            double endTime = ros::Time::now().toSec();

            while((endTime - beginTime) <= sec)
            {
                endTime = ros::Time::now().toSec();
                twistStamped.twist.angular.z = angular_stopped;
                twistStamped.twist.linear.x = linear_vel;
                //ros::Duration(0.01).sleep(); // sleep for 10 ms second
                PublishTwist();
            }
        }

        // At the moment the robot will turn around itself and follow the same line down again.
        void TurnRobotLeft(double sec)
        {
            double beginTime = ros::Time::now().toSec();
            double endTime = ros::Time::now().toSec();

            while((endTime - beginTime) <= sec)
            {
                endTime = ros::Time::now().toSec();
                twistStamped.twist.angular.z = angular_vel3;
                twistStamped.twist.linear.x = linear_stopped;
                ros::Duration(0.01).sleep(); // sleep for 10 ms second
                PublishTwist();
            }
        }

        // At the moment the robot will turn around itself and follow the same line down again.
        void TurnRobotRight(double sec)
        {
            double beginTime = ros::Time::now().toSec();
            double endTime = ros::Time::now().toSec();

            while((endTime - beginTime) <= sec)
            {
                endTime = ros::Time::now().toSec();
                twistStamped.twist.angular.z = -angular_vel3;
                twistStamped.twist.linear.x = linear_stopped;
                ros::Duration(0.01).sleep(); // sleep for 10 ms second
                PublishTwist();
            }
        }

        int ModNeg(double b, double m)
        {
            cout << "b is: " << b << endl;
            cout << "m is: " << m << endl;
            return b-m*floor(b/m);
        }

        double Deg2rad(double degree)
        {
            return degree*(double)(3.14159265359/180);
        }

        double Rad2Deg(double rad)
        {
            return rad*(double)(180/3.14159265359);
        }

        unsigned char CheckTurningAngleIMU(int desiredDegree, double beginDegree)
        {          
            //Added 180 to yaw and beginDegree have degrees between 0-360 degree
            double yawNorm = yaw + 180;
            double beginDegreeNorm = beginDegree + 180;

            //cout << "yawNorm is: " << yawNorm << endl;
            //cout << "beginDegreeNorm is: " << beginDegreeNorm << endl;
            //cout << "Degree in rad is: " << Deg2rad(beginDegreeNorm) << endl;

            Point2d v1, v2;
            v1.x = cos(Deg2rad(beginDegreeNorm));
            v1.y = sin(Deg2rad(beginDegreeNorm));
            v2.x = cos(Deg2rad(yawNorm));
            v2.y = sin(Deg2rad(yawNorm));

            //cout << "v1 is: " << v1.x << "," << v1.y << endl;
            //cout << "v2 is: " << v2.x << "," << v2.y << endl;
            // Find the angle between these two points.

            // http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
            double angle = Rad2Deg(acos(v1.dot(v2)));
            cout << "angle is: " << angle << endl;

            if (angle < desiredDegree)
            {
                cout << "Angle has not been reached: We still turn " << endl;
                //twistStamped.twist.angular.z = -angular_vel3;
                twistStamped.twist.angular.z = -angular_vel3;
                twistStamped.twist.linear.x = linear_stopped;
                return turning_state;
            }
            else
            {
                cout << "Done turning... " << endl;
                ros::Duration(2).sleep();
                return start_of_line_state;
            }
        }

        string IntToString (int a)
        {
            ostringstream temp;
            temp<<a;
            return temp.str();
        }

        void DebugState(unsigned char state)
        {
            string debugState;
            debugState = IntToString((int)state);
            cout << "DEBUG: state is: " << debugState << endl;
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
    ros::Rate loop_rate(100);

    // Here we initialize the state machine
    dc.current_state = follow_line_state;

    while (ros::ok())
    {

        // Check on if the user has entered an "e" into the system.
        // If this is true, we go to the state machine
        // else we dont do anything...
        if(dc.keyInput == 101)
        {
            //cout << " We received and e or 101" << endl;

            // State machine
            switch(dc.current_state)
            {
                case follow_line_state:
                    // When the robot is inside follow_line_state it use the data received from vision,
                    // to calculate the twist messeges
                    //cout << "r is " << dc.r << endl;
                    //cout << "theta is: " << dc.theta << endl;
                    dc.GetTwist(dc.r, dc.theta);

                    // Turning the robot to positive
                    //dc.twistStamped.twist.angular.z = 0.2;

                    // And then we publish that twist
                    dc.PublishTwist();

                    // Check if the robot has in end_of_line
                    dc.current_state = dc.CheckForEndOfLine();

                    // Check which state we are in
                    dc.DebugState(dc.current_state);

                break;

                case end_of_line_state:
                    // To be sure, that the robot do not just jump to end_of_line_state,
                    // in the transient response, we check again if we are in that state.
                    // Check if the robot has in end_of_line
                    dc.current_state = dc.CheckForEndOfLine();

                    // Check which state we are in
                    dc.DebugState(dc.current_state);

                    // Then stop the robot for 1 secound.
                    dc.StopRobot();
                    ros::Duration(sleepSec).sleep();

                    // Drive forward a little bit, to get out of the green line
                    dc.DriveRobotForward(runForwardSec);

                    // Store the yaw angle just before we start turning.
                    dc.beginDegree = dc.yaw;

                    // And then go to the turning_state
                    //dc.current_state = dc.WaitForTurning(2);
                    dc.current_state = turning_state;

                break;

                case turning_state:
                    // Turning the robot left around it own axis, hardcoded to approximately 180 degree
                    // dc.TurnRobotLeft(6.3);

                    // Check which state we are in
                    dc.DebugState(dc.current_state);

                    // Turn until the desired argument angle has been reached. Measured by the IMU
                    dc.current_state = dc.CheckTurningAngleIMU(175, dc.beginDegree);

                    // And then we publish that twist
                    dc.PublishTwist();

                break;

                case start_of_line_state:
                    //cout << "We are in start_of_line_state" << endl;

                    // Check which state we are in
                    dc.DebugState(dc.current_state);

                    // Drive forward a little bit, to get into the green line
                    //dc.DriveRobotForward(runForwardSec);

                    // Then stop the robot for 1 secound.
                    dc.StopRobot();
                    ros::Duration(sleepSec).sleep();

                    // And then go to the follow_line_state
                    dc.current_state = follow_line_state;
                break;

                default:
                    cout << "We got into default state" << endl;
            }
        }
        else
        {
            //cout << "Has not received any input yet" << endl;
        }

        //cout << "We do a spinOnce here" << endl;
        ros::spinOnce();

        // And wait
        loop_rate.sleep();

        // Make sure that we enter the call back functions onces

    }
    return 0;
}
