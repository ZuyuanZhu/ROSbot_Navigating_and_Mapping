/*
  We will create a node called stopper that will make 
  the robot move forward until it detects an obstacle in front of it.

  We will use the laser sensor data to achieve this. 
  
  Laser data is published to the topic /scan.
*/

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
	void moveForwardRight(const double turn_right_speed, const double move_forward_speed);	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void displayMessages();
};



Stopper::Stopper(){
	keepMoving = true;
	//Advertise a new publisher for the simulated robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	//Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("scan",1,&Stopper::scanCallback, this);
	
	//Subscribe to the odom topic
	odomSub = node.subscribe("odom", 1000, &Stopper::odomCallback, this);
}

//send a velocity command
void Stopper::moveForward(double forwardSpeed){
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = forwardSpeed; // // Drive forward at a given speed. The robot points up the x-axis.
	commandPub.publish(msg);
}

void Stopper::moveStop(){
	geometry_msgs::Twist msg; 
	msg.linear.x = FORWARD_SPEED_STOP;
	msg.angular.z = FORWARD_SPEED_STOP;
	commandPub.publish(msg);
}

void Stopper::moveForwardRight(const double turn_right_speed, const double move_forward_speed){
	geometry_msgs::Twist msg; 
	msg.angular.z = turn_right_speed;
	msg.linear.x = move_forward_speed;
	commandPub.publish(msg);
}

void Stopper::displayMessages(){
		ROS_INFO_STREAM("STAGE: " << STAGE);
		ROS_INFO_STREAM("rightRange: " << rightRange);  
		ROS_INFO_STREAM("leftRange: " << leftRange);  
		ROS_INFO_STREAM("frontRange: " << frontRange); 
		ROS_INFO_STREAM("rearRange: " << rearRange);
}

void Stopper::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){ 	
  	robotPoseX = odomMsg->pose.pose.position.x;
  	robotPoseY = odomMsg->pose.pose.position.y;
}


//process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	ros::Rate rate(20);		
	frontRange = scan->ranges[0];
	rightRange = scan->ranges[539];
	leftRange = scan->ranges[180];
	rearRange = scan->ranges[359];
	/*
	********************************************
	********** Put your code here **************
	********************************************
	*/
	
	// STAGE = 0, Initial stage, start moving towards the first gap, until:
	if (((rightRange < 0.38)||(frontRange < 1.0)) && STAGE==0){
		ROS_INFO("Turn Right");
		moveForwardRight(TURN_RIGHT_SPEED_HIGH, FORWARD_SPEED_HIGH);		
	}
	
	//If the robot is in the middle of the first gate, keep moving forward
	if( (0.2 <leftRange) && (leftRange < 0.66) &&  (0.2 < rightRange) && (rightRange < 0.45) && (1.8< frontRange) && (frontRange < 1.96)){
		STAGE = 1;
		moveStop();
		moveForward(FORWARD_SPEED_LOW);
		ROS_INFO("Arriving at the middle======");
		displayMessages();
		if(rearRange > 0.69){
            ROS_INFO("Arriving at the middle of gap 1, stop");
            moveStop();
            ros::shutdown();
        }
	}
						
}

void Stopper::startMoving(){
    int start = 0; 
	ros::Rate rate(20);
	ROS_INFO("Start moving");
	// keep spinning loop until user presses Ctrl+C
	while (ros::ok() && keepMoving){
		if(STAGE ==0){  			
			if(start < 10){ // start the robot, keep moving forward
			    moveForward(FORWARD_SPEED_HIGH);
			} 
			start++;
			displayMessages();			
		}
		ros::spinOnce(); // Allow ROS to process incoming messages
		rate.sleep();   // or loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{	// Initiate new ROS node named "stopper"
   ros::init(argc, argv, "stopper");  
   // Create new stopper object  
   Stopper stopper;
   // Start the movement
   stopper.startMoving();
	
   return 0;
}
