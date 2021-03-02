#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <time.h>
#include <iomanip>

using namespace std;


double robotPoseX; // robot pose X 
double robotPoseY;



class Stopper{
public:
// Tunable parameters
	constexpr const static double FORWARD_SPEED_LOW = 0.1;
	constexpr const static double FORWARD_SPEED_HIGH = 0.23;
	constexpr const static double FORWARD_SPEED_SHIGH = 0.4;
	constexpr const static double FORWARD_SPEED_STOP = 0;
	constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
	constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
	constexpr const static double TURN_RIGHT_SPEED_HIGH = -1.8;  
	constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
	constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
	unsigned int STAGE = 0;
	Stopper();
	void startMoving();
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
	void moveForward(double forwardSpeed);
	void moveStop();   //delete this line before passing to students
	void Stopper::moveForwardRight(const double turn_right_speed, const double move_forward_speed);
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void displayMessages();
};
