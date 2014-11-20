#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Trying to do the msg sending.
//#include <isa_project/num.h>
#include <isa_project/r_and_theta.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <stdio.h>
#include <vector>
#include <stdlib.h>     //for using the function sleep

#include "std_msgs/String.h"

using namespace cv;
using namespace std;

void test()
{
    cout << "this is a test" << endl;
}

vector<Mat> GetRGB(Mat image)
{
	vector<Mat> rgb_channel;
	split(image, rgb_channel);
	return rgb_channel;
}

vector<Mat> GetExcessiveRG(Mat R, Mat G, Mat B)
{
	vector<Mat> excessiveRG;
	//Create the excessive red
	Mat ExR = 1.4*R-G;
	excessiveRG.push_back(ExR);

	//Create the excessive green
	Mat ExG = 2*G-R-B;
	excessiveRG.push_back(ExG);

	// I dont do a excessive blue, so only returning the ExR and ExG. Red at 0 and Green at 1
	return excessiveRG;
}

Mat GetThreshold(Mat image, int thresholdValue)
{
	Mat treshold_img;
	threshold(image, treshold_img, thresholdValue, 255, CV_THRESH_BINARY);
	return treshold_img;
}

vector<Point> GetContoursAndXY(Mat image)
{
	// Find the countours
	vector<vector<Point> >contours;
	vector<Vec4i> hierarchy;
	findContours( image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	// Drawing the contours
	Mat drawing = Mat::zeros(image.size(), CV_8UC3 );
	RNG rng(12345);
	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

	int thickness = 2;
	int lineType = 8;

	// Compute mu moments and get center of mass
	Moments mu;
	std::vector< Point > circle_centroids;

	// Run trough all the contours
	for(int i = 0; i< contours.size(); i++ )
	{
		// Draw each contours
		drawContours(drawing, contours, i, color, thickness, lineType);

		// Create moments
		mu = cv::moments(contours[i], false);

		// By using central moments, the x and y coordinate of the center of mass can be found by given formulas
		Point2i mc( mu.m10/mu.m00 , mu.m01/mu.m00 );

		//cout << "mc.x is: " << mc.x << endl;
		if ((mc.x < 0)  || (mc.y) < 0)
		{
			// Dont put negative coordinats into the vector
			//cout << "Ops.. Negative coordinates: " << endl;
		}
		else // Else push the good coordiates into the vector
		{
			// Each x,y coordinate is pushed into the vector "circle_centroids"
			circle_centroids.push_back(mc);
		}
	}

	// Drawing the contours
	//imshow("contours drawing", drawing);
	//imwrite("/home/christian/Skrivebord/Images for rapport/drawing.png", drawing );
	//cout << "Number of countours is: " << circle_centroids.size() << endl;
	return circle_centroids;
}

Mat DrawCoordinates(Mat inputImage, vector< Point > circle_centroids, int B, int G, int R)
{
	Mat outputImage;
	inputImage.copyTo(outputImage);

	for(int i = 0; i< circle_centroids.size(); i++ )
	{
		int radius = 5;
		// Draw a filled circle(-1 as parameter for radius, 8 as linetype)
		circle(outputImage,circle_centroids[i],radius,Scalar(B, G, R),-1,8);
	}
	//imshow("Center of mass in original image", image);
	return outputImage;
}

int GetRandomIndex(int sizeOfVector)
{
	int randomIndex = 0;
	randomIndex = rand() % sizeOfVector;
	return randomIndex;
}

vector< Point > GetRandomCoordinates(vector< Point > circle_centroids)
{
	// Generate a random index
	vector< Point > twoRandomPoints;
	Point2f random1, random2;

	// Make sure the two points it not equal. So of course the first time they are equal, but after one run, hopefully the two points are differet...
	// If the two point are not different, i.e they are actually equal, then we try again in the while loop.
	while ( random1 == random2)
	{
		int randomIndex;
		int size = circle_centroids.size();

		randomIndex = GetRandomIndex(size);
		//cout << "Random Index is: " << randomIndex << endl;
		random1 = circle_centroids.at(randomIndex);
		//cout << "Random1 coordinate is: " << random1 << endl;

		//srand(time(NULL));
		randomIndex = GetRandomIndex(size);
		//cout << "Random Index is: " << randomIndex << endl;
		random2 = circle_centroids.at(randomIndex);
		//cout << "Random2 coordinate is: " << random2 << endl;
	}
	//cout << "The two points is not equal to each other anymore" << endl;
	twoRandomPoints.push_back(random1);
	twoRandomPoints.push_back(random2);
	return twoRandomPoints;
}

double GetLengthOfResidual(Point2f random1, Point2f random2, Point2f Q)
{
	// The vector12 is starting from random1 and goes in direction toward random2.
	Point2f vector12;
	vector12 = random2-random1;

	// Calculate the pendicular vector vector12_hat
	Point2f vector12_hat;
	vector12_hat.x = vector12.y;
	vector12_hat.y = vector12.x*-1;
	//cout << "vector12_hat is: " << vector12_hat << endl;

	// Calculate the length of vector vector12_hat
	double vector12_hat_magnitude = sqrt(vector12_hat.x*vector12_hat.x + vector12_hat.y*vector12_hat.y);
	//cout << "vector12_hat_length is: " << vector12_hat_magnitude << endl;

	// Calculate the unit vector
	Point2f vector12_hat_norm;
	vector12_hat_norm.x = (double)vector12_hat.x / vector12_hat_magnitude;
	vector12_hat_norm.y = (double)vector12_hat.y / vector12_hat_magnitude;
	//cout << "vector12_hat_norm is: " << vector12_hat_norm << endl;

	// Calculate the vector1Q, which is the vector from random1 to the point Q that should be decided if that is an inlier
	// or not. So looking in the vector with points is needed.
	Point2f vector1Q;
	double project_1Q_to_n;

	// Calculte the vector1Q
	vector1Q.x = Q.x - random1.x;
	vector1Q.y = Q.y - random1.y;

	// Calculate the projection of vector1Q downto vector12_hat_norm, e.g the dotproduct
	project_1Q_to_n = vector1Q.x * vector12_hat_norm.x + vector1Q.y * vector12_hat_norm.y;

	return project_1Q_to_n;
}

// Each time we go in here, we check a new line. So two new random1 and random2 points.
vector< vector < Point > > Function(Point2f random1, Point2f random2, vector< Point > circle_centroids, double pendicularDistance)
{
	vector< Point > inliers;
	vector< Point > bestPoints;
	vector< vector < Point> > vectorTrain;

	// Run trough the all the Q coordinates this line...
	//int threshold;
	for(int i = 0; i< circle_centroids.size(); i++ )
	{
		// Calculate the pendicular distance from the line that is spanded by random1 and random2 and towards the Q-point in circle_centroids
		double Qlength = GetLengthOfResidual(random1, random2, circle_centroids.at(i));

		// Say we wants to count a Q point as inliers, if and only if the length is below a given threshold
		if (abs(Qlength) < pendicularDistance)
		{
			// We have an inlier, so we store the Q point in the inlier vector
			inliers.push_back(circle_centroids.at(i));

			// Ex: Qpoint 4,15,17,22 was out of 23 an inlier. So the size of the vector should be 4.

			//circle(temp,circle_centroids.at(i),5,Scalar(255, 100, 100),-1,8);
			//imshow("Center of mass in original image", image);
		}
		else
		{
			// We have an outlier
			//outlierCounter++;
			//circle(image,circle_centroids.at(i),5,Scalar(255, 255, 255),-1,8);
		}
		//cout << "vector1Q is: " << project_1Q_to_n << endl;
		//cout << "vector12_hat_norm is: " << vector12_hat_norm << endl;
		//cout << "project_1Q_to_n is: " << project_1Q_to_n << endl;
		//cout << "The number of inliers is: " << inlierCounter << endl;
		//cout << "The number of outliers is: " << outlierCounter << endl;
		// Then project the vector1Q vector down to the vector12_hat_norm vector
	}

	// Store the random1 and random2 as the best points
	bestPoints.push_back(random1);
	bestPoints.push_back(random2);

	// Store the vectors of inliers and bestPoints into the return vector called vectorTrain
	vectorTrain.push_back(inliers);
	vectorTrain.push_back(bestPoints);

	// For loop done. Has been trough all the Q-points in the circle_centroid
	// So I would like to push the random1 and random2 at the end of inlier vector
	// so I can get thoese coordinates to draw the final line at the end.
	// To get the total number of inliers, I just take the size of inliers.size() - 2,
	// since we added two "togvogne til godstoget" like int numberOfInliers = inliers.size()-2;
	// Could also just say return a vector of a vector with points.
	// vector < vector < points > >
	// Then the first vector that is returned is all the inliers, and the next vector is just the bestPoint1+2.
	
	return vectorTrain;
}

vector< vector < Point > > RANSAC_CLH(int triesLines, double pendicularDistance, vector< Point > circle_centroids)
{
	vector< vector < Point > > resultFromRANSAC;
	vector< Point > randomPoints;
	vector< Point > bestInliers;
	vector< Point > bestPoints;
	vector< vector < Point > > resultFromFunction;

	int inlierThreshold = 0;
	for(int i = 0; i < triesLines; i++) // Each run is a new line we are testing for inliers in the same image
	{
		// We get randomly (x1,y1) and (x2,y2) and store this in the randomPoints vector.
		randomPoints = GetRandomCoordinates(circle_centroids);
		//cout << "The two random points is:" << randomPoints << endl;

		resultFromFunction = Function(randomPoints.at(0), randomPoints.at(1), circle_centroids, pendicularDistance);
		//cout << "What do I get? " << resultFromTrain.at(0) << endl; // This is the inlier vector
		//cout << "What do I get? " << resultFromTrain.at(1) << endl; // This is the bestPoints vector.

		if(resultFromFunction.at(0).size() > inlierThreshold) // So first time, yes even that we get a line with only 4 inliers, 4 is stil bigger than 0
		{
			// So now we update our bestPoints and the best inliers
			// Now we raise the threshold, so if a new line has lower inliers, we dont overwrite the bestpoints and inliervector
			inlierThreshold = resultFromFunction.at(0).size();

			// And store the best Q-points from the inlier vector
			bestInliers = resultFromFunction.at(0);

			// And store the best start-stop points that should be used to draw the finish line after the all the tries in RANSAC.s
			bestPoints = resultFromFunction.at(1);
		}
		else
		{
			// Try a new line until you get tired, RANSAC...
			// You have the number of "triesLines" tries left...
		}
	}
	// Push bestInliers into the return vector of the RANSAC algorithm
	resultFromRANSAC.push_back(bestInliers);

	// Push bestPoints into the return vector of the RANSAC algorithm
	resultFromRANSAC.push_back(bestPoints);

	return resultFromRANSAC;
}

void DrawResult(Mat image, Point pointA, Point pointB, int pendicularDistance)
{
	Point leftMargin1, leftMargin2, rightMargin1, rightMargin2;
	leftMargin1.x = pointA.x;
	leftMargin1.y = pointA.y + (int)pendicularDistance;
	leftMargin2.x = pointB.x;
	leftMargin2.y = pointB.y + (int)pendicularDistance;

	rightMargin1.x = pointA.x;
	rightMargin1.y = pointA.y - (int)pendicularDistance;
	rightMargin2.x = pointB.x;
	rightMargin2.y = pointB.y - (int)pendicularDistance;

	line(image, leftMargin1, leftMargin2, Scalar(255, 0, 0), 2, 8,0);
	line(image, rightMargin1, rightMargin2, Scalar(255, 0, 0), 2, 8,0);

	// Draw the middle line between the start and stop points, aka. the bestPoints
	line(image, pointA, pointB, Scalar(0, 0, 255), 2, 8,0);

	// Draw the start and stop points, aka. the bestPoints
	circle(image,pointA, 5, Scalar(0, 255, 0),-1,8); 		// Draw the start point green
	circle(image,pointB, 5, Scalar(0, 0, 255),-1,8);		// Draw the end point red
}

vector<double> GetAngleInDegree(Point bestPoint1, Point bestPoint2)
{
	vector<double> returnVector;

	// What I have to do is find the coordinate of where the r vector ends.
	double x = bestPoint2.x - bestPoint1.x;
	double y = bestPoint2.y - bestPoint1.y;

	//cout << "bestPoint1 is: (" << bestPoint1.x << "," << bestPoint1.y << ")" << endl;
	//cout << "bestPoint2 is: (" << bestPoint2.x << "," << bestPoint2.y << ")" << endl;

	// Put the x,y coordinate into the vector
	returnVector.push_back(x);
	returnVector.push_back(y);

	//cout << "x is: " << x << endl;
	//cout << "y is: " << y << endl;
	// See more on atan2 on http://en.wikipedia.org/wiki/Atan2
	// The if, else if etc is defined out from the wiki page.

	double theta = 0.0;
	if (x > 0)
	{
		theta = atan2(y,x);
		//cout << "x is: " << x << " and y is: " << y << endl;
		//cout << "x > 0 so, theta is: " << theta << endl;
	}
	else if (y >= 0 && x < 0)
	{
		theta = atan2(y,x) + (double)(M_PI);
		//cout << "y >= 0 && x < 0 so, theta is: " << theta << endl;
	}
	else if (y < 0 && x < 0)
	{
		theta = atan2(y,x) - (double)(M_PI);
		//cout << "y < 0 && x < 0 0 so, theta is: " << theta << endl;
	}
	else if (y > 0 && x == 0)
	{
		theta = (double)(M_PI/2);
		//cout << "y > 0 && x == 0 so, theta is: " << theta << endl;
	}
	else if (y < 0 && x == 0)
	{
		theta = -1*(double)(M_PI/2);
		//cout << "y < 0 && x == 0 so, theta is: " << theta << endl;
	}
	else
	{
		// Undefined... return
		cout << "Error" << endl;
	}

	// Convert into degree
	theta = theta* (180/M_PI);
	//cout << "So the angle right now is: " << theta << endl;

	// In the image plane where origo is upper left corner. Going one coloum
	// to right increment x.
	// Going one row down increment y.
	// Got it?

	// Test shows that a line which is angled in appr. 55 degrees like this:
	// |
	// |       /
	// |      /
	// |     /
	// |    /
	// |   /
	// |_____________

	// Gave result as: - 55.or 305
	// Where we want it to be 55, since we meassure it from the horisontal x axsis.
	// So we want to meassure from 0 to 180
	// That means if the angle is vertical in the image plane, the angle will jump between 0 and 180
	// This is okay, since the robot should never stand in that position, like pendicular to the maize rows.
	// The robot should drive parallel to the maize rows, otherwise something is really wrong :)
	// So we expect to have a robot driving parallel to the rows, means a angle appr. 90 degree and will
	// in real lift propperly be in range 45 to 135.

	// Test shows that a line which is angled in appr. 125 degrees like this:
	// |
	// |   \
	// |    \
	// |     \
	// |      \
	// |       \
	// |_____________

	// Gave result as: - 305 or 55

	// if theta is between -360 and -270 OR between 0 and 90.
	// e.g theta = -220

	// If the angle is between -360 and -270 or the angle is from 0 to 90,
	if( (theta >= -360 && theta <= -270) ||  (theta >= 0 && theta <= 90) )
	{
		// All the negative angles, i.e lines that os trough the 2. and 4. quadrant (kind of, if all the lines are trough a
		// vertual 0.0)
		// Then we have to see what kind of modification we need to do to the angle.

		// If the angle was between -360 and -270, then the transformation is to get the absolute value of angle
		// and subtract 360 degrees to get between 0 and -90
		if (theta >= -360 && theta <= -270)
		{
			//cout << "theta is: " << theta << endl;
			theta = abs(theta)-180;
			//cout << "The angle is correct to: " << theta << endl;

		}
		else // Means (theta >= 0 && theta <= 90)
		{
			//cout << "theta is: " << theta << endl;
			//theta = theta*-1;
			theta = abs(theta-180);
			//cout << "The angle is correct to: " << theta << endl;
		}
	}
	else if( (theta >= -90 && theta <= 0) ||  (theta >= 270 && theta <= 360) )
	{
		// Means (theta >= -90 && theta <= 0) ||  (theta >= w70 && theta <= 360) )
		// All the negative angles, i.e lines that os trough the 1. and 3. quadrant (kind of, if all the lines are trough a
		// vertual 0.0)
		// Then we have to see what kind of modification we need to do to the angle.

		// If the angle was between -90 and 0, then the transformation is to get the absolute value of angle
		if (theta >= -90 && theta <= 0)
		{
			//cout << "theta is: " << theta << endl;
			theta = theta*-1;
			//cout << "The angle is correct to: " << theta << endl;
		}
		// Else if the angle is between 370 and 360, then then we correct by get the absolute value after theta has been subtracted 360 degree
		else // means (theta >= 270 && theta <= 360)
		{
			//cout << "theta is: " << theta << endl;
			theta = abs(theta-360);
			//cout << "The angle is correct to: " << theta << endl;
		}
	}
	else
	{
		// We should not get in here...
		cout << "Error in transformation " << endl;
	}

	// Push the angle at the last 3. parameter in the returnVector.
	returnVector.push_back(theta);

	//cout << "Theta is: " << theta << endl;
	return returnVector;
}
// http://wikicode.wikidot.com/get-angle-of-line-between-two-points#sthash.HDvFgJQ9.dpuf

double GetR(Point bestPoint1, Point bestPoint2)
{
	// Get distance from origo to the line pendicually.
	double r;
	Point origo;
	origo.x = 0;
	origo.y = 0;

	r = abs(GetLengthOfResidual(bestPoint1, bestPoint2, origo));

//	double radTheta = (M_PI/180)* theta;

	// Convert it back to radians

//	cout << "bestPoint1.x: " << bestPoint1.x << "\t bestPoint1.y " << bestPoint1.y << endl;
//	cout << "bestPoint2.x: " << bestPoint2.x << "\t bestPoint2.y " << bestPoint2.y << endl;
	//r = bestPoint1.x*cos(radTheta) + bestPoint1.y*sin(radTheta);

//	cout << "x is:" << x << endl;
//	cout << "y is:" << y << endl;
//	cout << "radTheta is:" << radTheta << endl;
//	r = x*cos(theta) + y*sin(theta);
	return r;
}

void AddInlierText(Mat image, int xlocation, int ylocation ,string text, int number)
{
	// Add the number of inliers to the images
	Point textLocation;
	int sizeOfText = 1;
	int thicknessOfText = 2;
	int lineType = 8;

	// Typecaster number to int

	char txt[16];
	sprintf(txt, "%d", number);

	string text2 = txt;

	// Concatunate the two strings
	text = text + text2;
	textLocation.x = xlocation;
	textLocation.y = ylocation;
	//putText(RANSAC_image,"Test",textLocation,8,CV_FONT_HERSHEY_SIMPLEX,Scalar(0, 0, 255),4,8,false);
	putText(image, text, textLocation, CV_FONT_HERSHEY_SIMPLEX, sizeOfText, Scalar(255, 0, 0), thicknessOfText, lineType);
	//imshow("RANSAC result", image);
}

VideoCapture OpenCamera(int device)
{
	VideoCapture video(device);

	//check if video device has been initialised
	if (!video.isOpened()) { //check if video device has been initialised
	cout << "cannot open camera";
	}

	return video;
}

Mat GetImageFromCamera(VideoCapture video)
{
	// Create a Mat objet frame
	Mat cameraFrame;

	// Getting the latest image from camera stored in the variable cameraFrame.
	video.read(cameraFrame);

	// Returning the camera
	return cameraFrame;
}

void CountDownSec(int sec)
{
    cout << "We count down " << sec << " secounds..." << endl;
    for(int i = sec; i > 0; i--)
    {
        cout << i << endl;
        waitKey(1000);
    }
    cout << "Starting..." << endl;
}
///////////////////////////////////////////////////////////
// The main function
///////////////////////////////////////////////////////////

class TestClass
{
    public:
    void coolSaying()
    {
        cout << "TEEEESSSTTTT of function" << endl;
    }


};

int main(int argc, char **argv)
{
    TestClass object;
    object.coolSaying();

	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "vision_node");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

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
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	//ros::Publisher test_pub = n.advertise<isa_project::num>("/testing_num", 1);
	ros::Publisher r_and_theta_pub = n.advertise<isa_project::r_and_theta>("/r_and_theta", 1);
	//ros::Rate loop_rate(10);
	
	//isa_project::num msg;
	isa_project::r_and_theta msg;

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
	int count = 0;
	double r;
	double theta;
	int iAngle = 180;
    int thresholdValue = 50;
    int xt = 0;
    int yt = 0;

    // Open the webcam
    VideoCapture video = OpenCamera(1);
    // Count down the secounds before running to stabalize the camera

    bool flag = true;
    while (ros::ok())
    {
        int xt_offset;
        int yt_offset;
        // Note: 29/10-2014
        // The camera input is not enabled yet, since decision node is not implemented yet
        // I want to test the motors

        Mat inputImage;
        //inputImage = GetImageFromCamera(video);
        inputImage = imread("/home/christian/workspace_eclipseLuna/DisplayImage/src/indoorFinal1.jpg", CV_LOAD_IMAGE_COLOR);

        if(flag)
        {
            xt_offset = inputImage.cols;
            yt_offset = inputImage.rows;
            xt = xt_offset;
            yt = yt_offset;
            flag = false;
        }

        // Here we do the rotation of the image
        createTrackbar("Angle", "RANSAC_image", &iAngle, 360);
        createTrackbar("Translation x", "RANSAC_image", &xt, 2*inputImage.cols);
        createTrackbar("Translation y", "RANSAC_image", &yt, 2*inputImage.rows);
        Mat matRotation = getRotationMatrix2D( Point(inputImage.rows/2, inputImage.cols/2), (iAngle - 180), 1 );

        // Do some translation, by multipliing the rotation 2,3 with and 3,3, matrix to get the 2,3 translated and rotated matrix
        Mat matTranslation = (cv::Mat_<double>(3,3) <<     1,0,1*(xt-xt_offset),
                                                           0,1,1*(yt-yt_offset),
                                                           0,0,1);

        // Do the multiplication here to translate the image
        Mat mult_M = matRotation * matTranslation;

        // Rotate the image after the translation.
        Mat imgRotatedAndTranslated;
        //warpAffine(inputImage, imgRotated, matRotation, inputImage.size() );
        warpAffine(inputImage, imgRotatedAndTranslated, mult_M, inputImage.size() );

        // Copy/overwrite the imgRotatedAndTranslated over into inputImage
        imgRotatedAndTranslated.copyTo(inputImage);

        // Resize scale
        int resizeScale = 3;
        // Resize the image
        Size size(inputImage.cols/resizeScale,inputImage.rows/resizeScale);//the dst image size,e.g.100x100
        resize(inputImage,inputImage,size);//resize image

        // Split the input image into R,G and B.
        Mat R =	GetRGB(inputImage).at(2);
        Mat G =	GetRGB(inputImage).at(1);
        Mat B =	GetRGB(inputImage).at(0);

        //Create the excessive green, red and the difference
        Mat ExG = GetExcessiveRG(R,G,B).at(1);
        Mat ExR = GetExcessiveRG(R,G,B).at(0);
        Mat ExGR = ExG-ExR;

        //Create the threshold - hardcodede at the moment 17/11-2014.
        createTrackbar("Threshold", "Thresholded image", &thresholdValue, 255);
        Mat treshold_img = GetThreshold(ExGR, thresholdValue);
        imshow("Thresholded image", treshold_img);


        // Do morphology to the image to limit the search with ransac.
        Mat erode_img, dilate_img;
        int iterations = 1; // Tested with 1
        erode(treshold_img,erode_img,Mat(),Point(-1, -1),iterations,BORDER_CONSTANT,morphologyDefaultBorderValue());
        dilate(erode_img, dilate_img,Mat(),Point(-1, -1),iterations,BORDER_CONSTANT,morphologyDefaultBorderValue());

        // Get the contours and find the centroid x,y point, circle_centroids, for each contour
        vector< Point > circle_centroids;
        circle_centroids = GetContoursAndXY(erode_img);
        cout << "number of circle_centroids is: " << circle_centroids.size() << endl;

        // If there is not too many contours, then the image is relative noise free.
        if(circle_centroids.size() < 50)
        {
            // Draw the central coordinates for each contours on the final RANSAC image
            Mat RANSAC_image;
            RANSAC_image = DrawCoordinates(inputImage, circle_centroids, 255, 255, 255); // R,B,G. Between 0 and 255

            // Make sure that there is at least two contours/point, so the RANSAC algorithm
            // can work. Do not make sense to find a line between 0 or 1 point.

            if(circle_centroids.size() < 2)
            {
                //cout << "Hey the size is < 2" << endl;
                //cout << "so we skip the image..." << endl;
            }
            else
            {
                //cout << "Hey the size is >= 2" << endl;
                //cout << "so we continue..." << endl;

                // Finding lines in the images using RANSAC
                int triesLines = 1000; // Tested with 1000
                double pendicularDistance = 20.0; // Tested with 10.0
                vector < vector < Point > > resultFromRANSAC;
                resultFromRANSAC = RANSAC_CLH(triesLines, pendicularDistance, circle_centroids);

                vector<Point> bestPoints;
                vector<Point> bestInliers;
                bestInliers = resultFromRANSAC.at(0);
                bestPoints = resultFromRANSAC.at(1);

                // Draw the inliers
                RANSAC_image = DrawCoordinates(RANSAC_image, bestInliers, 255, 0, 0); // B,G,R. Between 0 and 255
                //cout << "Number of inliers is: " << bestInliers.size() << endl;
                //cout << "qt" << endl;

                // Draw the best fitted line for each image
                DrawResult(RANSAC_image, bestPoints.at(0), bestPoints.at(1), pendicularDistance);

                vector<double> returnVector;
                returnVector = GetAngleInDegree(bestPoints.at(0), bestPoints.at(1));
                theta = returnVector.at(2);
                r = GetR(bestPoints.at(0), bestPoints.at(1));

                // Add the values directly in the input image
                AddInlierText(RANSAC_image, RANSAC_image.rows/15, RANSAC_image.rows/15, "r: ", (int)r);
                AddInlierText(RANSAC_image, RANSAC_image.rows/8, RANSAC_image.rows/8, "theta: ", (int)theta);

                // Plot the r and theta in the terminal
                //cout << "r: " << r << "\t" << " theta (degree): " << theta << endl;
            }

            imshow("RANSAC_image", RANSAC_image);
            waitKey(1); // Wait 1 ms to make the imshow have time to show the image

            // Publish the r and theta trough ROS
            msg.r = r;
            msg.theta = theta;

            // And then we send it on the test_pub topic
            r_and_theta_pub.publish(msg);

            // Spin once
            //ros::spinOnce();

            // sleep for the time remaining to let us hit our 10hz publish rate.
            //loop_rate.sleep();

        }
        else
        {
            cout << "We are waiting for the camera" << endl;
        }
	}

	return 0;
}

//////////////////////////////
// Garbage
//////////////////////////////

/**
		  * This is a message object. You stuff it with data, and then publish it.
		  */

//		// Meassage string object
//		std_msgs::String msg;
//
//		// String stream object
//		std::stringstream ss;
//
//		// Put text and data into the stringstream object ss
//		ss << "r:" << r << " " << "theta:" << theta;
//
//		// Store the object ss as a string as data in msg object.
//		msg.data = ss.str();
//
//		// This act like cout/printf. Only to show in console what we send. If we outcomment this
//		// the listener_node will still be able to get what we publish. The only difference
//		// is that we dont see what we send if we dont do ROS_INFO.
//		// Or we could do cout instead...
//		ROS_INFO("%s", msg.data.c_str());

		 /**
		  * The publish() function is how you send messages. The parameter
		  * is the message object. The type of this object must agree with the type
		  * given as a template parameter to the advertise<>() call, as was done
		  * in the constructor above.
		  */

		// Now we actually broadcast the message to anyone who is connected.
		//chatter_pub.publish(msg);
