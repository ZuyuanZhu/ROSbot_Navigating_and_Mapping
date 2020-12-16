#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <time.h>
#include <iomanip>

using namespace std;

ofstream odomVelFile;
ofstream odomMapFile;

//ros::Time beginTime;  // start time 
//ros::Time currentTime; //current time
//ros::Duration moment; // moment = currentTime - beginTime

double beginTime;  // start time 
double currentTime; //current time
double moment; // moment = currentTime - beginTime
double robotPoseX; // robot pose X 
double robotPoseY;
double robotHeadAngle; // robot heading angle

struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}


class Stopper {
public:
// Tunable parameters
	constexpr const static double FORWARD_SPEED_LOW = 0.1;
	constexpr const static double FORWARD_SPEED_HIGH = 0.2;
	constexpr const static double FORWARD_SPEED_SHIGH = 0.4;
	constexpr const static double FORWARD_SPEED_STOP = 0;
	constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
	constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
	constexpr const static double TURN_RIGHT_SPEED_HIGH = -2.4;  //-1.8
	constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
	constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
	constexpr const static double LEFT_SCAN_ANGLE_RAD = -90.0/180*M_PI;
	constexpr const static double RIGHT_SCAN_ANGLE_RAD = +90.0/180*M_PI;
	constexpr const static double FRONT_SCAN_ANGLE_RAD = 0.0/180*M_PI;
	constexpr const static float MIN_PROXIMITY_RANGE_M = 0.4; //Should be smaller than sensor_msgs::LaserScan::range_max
	unsigned int STAGE = 0;
	Stopper();
	void startMoving();
	void keepMoveAlongWall(double range2Wall);
private:
	ros::NodeHandle	node;
	ros::Publisher commandPub;	// Publisher to	the	robot's	velocity command topic
	ros::Subscriber laserSub;	// Subscriber to the robot's laser scan topic
	ros::Subscriber odomSub;	// Subscriber to the robot's odom topic
	bool keepMoving;	// Indicates whether the robot should continue moving
	float frontRange = 100.0;  // laser range distance, initialize at a long distance. 
	float leftRange = 100.0;
	float rightRange = 100.0;
	float rearRange = 100.0;
	constexpr const static int FRONT_ANGLE_RAD = 0.0;
	constexpr const static int LEFT_ANGLE_RAD = -90.0/180*M_PI;
	constexpr const static int RIGHT_ANGLE_RAD = 90.0/180*M_PI;
	void moveForward(double forwardSpeed);
	void moveStop();   //delete this line before passing to students
	void moveLeft(double turn_left_speed);   //delete this line before passing to students
	void moveRight(double turn_right_speed = TURN_RIGHT_SPEED_HIGH);
	void keepMiddleTorward2ndGap(unsigned int rightClosestRangeIndex, unsigned int leftClosestRangeIndex); // keep in the middle when moving torwards the 2nd gaps
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void displayMessages();
	void transformMapPoint(ofstream& fp, double _obstacleRange, double _sonarX, double _sonarY, double _sonarTh, double _robotTh, double _globalX, double _globalY);  //coordinate transfer for mapping
	//int Stopper::findClosestRangeIndex(int indexRangeMin, int indexRangeMax, const sensor_msgs::LaserScan::ConstPtr& laserScan); not working
};
