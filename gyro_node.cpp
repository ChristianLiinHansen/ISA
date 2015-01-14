#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Trying to do the msg sending.
#include <isa_project/orientation.h>

// This is to subscribe to sensor_msg::Imu
#include "sensor_msgs/Imu.h"

// This is to use tf functions
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <stdio.h>
#include <vector>
#include <stdlib.h>     //for using the function sleep
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

class ImuConverter
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
    ros::Subscriber imu_sub;



    ///////////////////////////////////////////////////////////////////
    // Private member functions
    ///////////////////////////////////////////////////////////////////

    public:

        ///////////////////////////////////////////////////////////////////
        // Public variables
        ///////////////////////////////////////////////////////////////////

        double degree;

        // Decklare the message
        isa_project::orientation msg_orientation;

        // Declare the publishers
        ros::Publisher orientation_pub;

        ///////////////////////////////////////////////////////////////////
        // Public member functions
        ///////////////////////////////////////////////////////////////////

        ImuConverter()
        {
            // Initialize the subscribers in the constructor
            imu_sub = n.subscribe("/fmInformation/imu", 1000, &ImuConverter::imu_CallBack, this);

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
            orientation_pub = n.advertise<isa_project::orientation>("/orientation", 1);
        }

        ~ImuConverter()
        {

        }

        void imu_CallBack(const sensor_msgs::Imu::ConstPtr& msg)
        {
            // Taken from http://answers.ros.org/question/36977/invalid-arguments-convert-from-quaternion-to-euler/
            // Convert quaternion to RPY.
            tf::Quaternion q;
            tf::quaternionMsgToTF(msg->orientation, q);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // Store the vawlue of yaw into
            degree = getRad2Degre(yaw);
        }

        double getRad2Degre(double rad)
        {
            return rad * (180.0/3.141592653589793);
        }

};

int main(int argc, char **argv)
{
    // Initialize this ROS node
    ros::init(argc, argv, "gyro_node");

    // Initialize the class ImuConverter
    ImuConverter ic;

    // Try to set the loop rate down to avoid the watchdog timer to set linary and velocity to zero.
    ros::Rate loop_rate(30);

    // Publisher
    //ros::Publisher orientation_pub = n.advertise<isa_project::orientation>("/orientation", 1);

    // Decklare the message
    //isa_project::orientation msg_orientation;

    while (ros::ok())
    {
        // Take the stored orientation around the z axes in degrees and stored in the msg_orientation messages
        ic.msg_orientation.yaw = ic.degree;
        cout << "And the degree was again2: " << ic.msg_orientation.yaw << endl;

        // And then we send it on the /orientation topic
        ic.orientation_pub.publish(ic.msg_orientation);

        // And wait
        loop_rate.sleep();

        // Make sure that we enter the call back functions onces
        ros::spinOnce();
	}
	return 0;
}
