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

//class ImuConverter
//{
//    public:

//        ///////////////////////////////////////////////////////////////////
//        // Public variables
//        ///////////////////////////////////////////////////////////////////

//        double yaw, degree;

//        ///////////////////////////////////////////////////////////////////
//        // Public member functions
//        ///////////////////////////////////////////////////////////////////

//        ImuConverter()
//        {
//            //cout << "Calling the constructor" << endl;
//            degree = getRad2Degre(yaw);
//        }

//        ~ImuConverter()
//        {
//            //cout << "Calling the deconstructor" << endl;
//        }

//        double getRad2Degre(double rad)
//        {
//            return rad * (180.0/3.141592653589793);
//        }

////        double getDegreeInRange(int startValue, int endValue, double degree)
////        {

////            return
////        }

//    private:

//        ///////////////////////////////////////////////////////////////////
//        // Private variables
//        ///////////////////////////////////////////////////////////////////


//        ///////////////////////////////////////////////////////////////////
//        // Private member functions
//        ///////////////////////////////////////////////////////////////////
//};

void imu_CallBack(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Taken from http://answers.ros.org/question/36977/invalid-arguments-convert-from-quaternion-to-euler/
    // Convert quaternion to RPY.
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
    cout << "yaw: \t" << yaw << endl;

    // Instantiate the call ImuConverter to create the object ic
    //ImuConverter ic;
    //ic.yaw = yaw;

    //cout << "yaw is in degree:\t" << yaw * (180.0/3.141592653589793) << endl;
}

int main(int argc, char **argv)
{
    // Initialize this ROS node
    ros::init(argc, argv, "gyro_node");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

    // Try to set the loop rate down to avoid the watchdog timer to set linary and velocity to zero.
    ros::Rate loop_rate(30);

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

    // Subscriber
    ros::Subscriber imu_sub = n.subscribe("/fmInformation/imu", 1000, imu_CallBack);

    // Publisher
    ros::Publisher orientation_pub = n.advertise<isa_project::orientation>("/orientation", 1);

    // Decklare the message
    isa_project::orientation msg_orientation;

    while (ros::ok())
    {
        // Just for testing that the topic /orientation contains 0.23. Yes it works.
        msg_orientation.yaw = 0.23;

        // And then we send it on the test_pub topic
        orientation_pub.publish(msg_orientation);

        // And wait
        loop_rate.sleep();

        // Make sure that we enter the call back functions onces
        ros::spinOnce();
	}

	return 0;
}
