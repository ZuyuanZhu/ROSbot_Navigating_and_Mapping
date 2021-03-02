/*
  We will create a node called stopper that will make 
  the robot move forward until it detects an obstacle in front of it.

  We will use the laser sensor data to achieve this. 
  
  Laser data is published to the topic /scan.
*/

#include "../include/Stopper.h"

double trs = -2.8;
double fs = 0.27;
double mfs = 0.28;


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

void Stopper::moveLeft(double turn_left_speed){
	geometry_msgs::Twist msg; 
	msg.angular.z = turn_left_speed;
	commandPub.publish(msg);
}

void Stopper::moveRight(double turn_right_speed){
	geometry_msgs::Twist msg; 
	msg.angular.z = turn_right_speed;
	msg.linear.x = fs;
	commandPub.publish(msg);
}





/*   not working
int Stopper::findClosestRangeIndex(int indexRangeMin, int indexRangeMax, const sensor_msgs::LaserScan::ConstPtr& laserScan){
	float closestRange = laserScan->ranges[indexRangeMin];
	int closestRangeIndex = indexRangeMin;
	for (int currIndex = indexRangeMin + 1; currIndex <= indexRangeMax; currIndex++){
		if (laserScan->ranges[currIndex] < closestRange){
			closestRange = laserScan->ranges[currIndex];
			closestRangeIndex = currIndex;
		}
	}
	ROS_INFO_STREAM("closestRangeIndex" << closestRangeIndex);
	return closestRangeIndex;
}

*/

void Stopper::displayMessages(){
		ROS_INFO_STREAM("STAGE: " << STAGE);
		ROS_INFO_STREAM("rightRange: " << rightRange);  
		ROS_INFO_STREAM("leftRange: " << leftRange);  
		ROS_INFO_STREAM("frontRange: " << frontRange); 
		ROS_INFO_STREAM("rearRange: " << rearRange);
		ROS_INFO_STREAM("robotHeadAngle: " << robotHeadAngle); 
		ROS_INFO_STREAM("robotPoseX: " << robotPoseX); 
		ROS_INFO_STREAM("robotPoseY: " << robotPoseY); 
}









void Stopper::odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg){
	//ROS_INFO("Seq: [%d]", odomMsg->header.seq);
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odomMsg->pose.pose.position.x,odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y, odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odomMsg->twist.twist.linear.x,odomMsg->twist.twist.angular.z);
	//currentTime = ros::Time::now().toSec();
	//moment = currentTime - beginTime;
	odomVelFile << odomMsg->twist.twist.linear.x << "\n"; // record the moment and speed, for velocity figure
	
	Quaternion robotQuat;
	robotQuat.x = odomMsg->pose.pose.orientation.x;
	robotQuat.y = odomMsg->pose.pose.orientation.y;
	robotQuat.z = odomMsg->pose.pose.orientation.z;
	robotQuat.w = odomMsg->pose.pose.orientation.w;
	EulerAngles robotAngles;	
  	robotAngles = ToEulerAngles(robotQuat);
  	robotHeadAngle = robotAngles.yaw;
  	
  	robotPoseX = odomMsg->pose.pose.position.x;
  	robotPoseY = odomMsg->pose.pose.position.y;
  	//transformMapPoint(odomMapFile, frontRange, 0, 0, 0, robotAngles.yaw, robotPoseX, robotPoseY);
  	//transformMapPoint(odomMapFile, rightRange, 0, 0, -1.57, robotAngles.yaw, robotPoseX, robotPoseY);
  	//transformMapPoint(odomMapFile, leftRange, 0, 0, 1.57, robotAngles.yaw, robotPoseX, robotPoseY);
}






//process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	//	Find the closest range between the defined minimum and maximum angles
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

	// STAGE = 0, Initial stage, start moving towards the first gap before turing right
	if (((rightRange < 0.38)||(frontRange < 1.0)) && STAGE==0){
		ROS_INFO("Turn Right");
		moveRight(trs);
		displayMessages();
	}
	
	// finished first turning, if robot head angle is 0, i.e., heading towards down
	if( (-0.09 < robotHeadAngle && robotHeadAngle < 0.09) && (frontRange < 2.5 ) &&( rearRange> 1)){
	     STAGE = 1; 
	    // stop turning right, move forward
	    ROS_INFO_STREAM("==========stop turning right ");
	    moveStop();
	    //ros::shutdown();
	    moveForward(FORWARD_SPEED_LOW);
	    displayMessages();
	    // arriving at the centre of gap 1 (0.5, 1.575), stop
	    if ( robotPoseX > 0.45  ) {
	        moveStop();
	        ros::shutdown();
	    }
	    
	    if(( robotPoseX > 0.45 && robotPoseX < 0.55) && (robotPoseY > 1.525 && robotPoseY < 1.625)){
	    moveStop();
	    }
	}
	//If the robot is in the middle of the first gate, keep moving forward
	/*
	if( (0.2 <leftRange) && (leftRange < 0.64) &&  (0.2 < rightRange) && (rightRange < 0.35) && (1.8< frontRange) && (frontRange < 1.9)){
		STAGE = 1;

		moveForward(FORWARD_SPEED_LOW);
		//displayMessages();
	}
	
	if( (frontRange > 1.3) && (STAGE ==1)){  // Move through 1st gap
		moveForward(FORWARD_SPEED_LOW);
		//displayMessages();
	}
	if((frontRange < 1.3)&&(STAGE ==1)){ //Moving forward ends
		STAGE =2;
		//displayMessages();	
	}	
	if((frontRange < 1.3) && (STAGE ==2)){  //turn right, torwarding the 2nd gap
		moveRight(TURN_RIGHT_SPEED_LOW);
		//displayMessages();
		if((((rightRange > 2.1)||(rightRange < 0.65))&&(frontRange > 0.89)&&(leftRange > 0.56))||((rightRange > 1.7) && (leftRange > 0.54) && (frontRange > 1.15))){ //laser noise makes it hard to find the right conditions
			STAGE = 3;				
		}
	}
	*/
							
}

void Stopper::startMoving(){
	ros::Rate rate(20);
	ROS_INFO("Start moving");
	moveForward(mfs);
	//odomVelFile.open("/home/zuyuan/ros_ws/src/tutorial_pkg/odomVelData.csv",ios::out | ios::app);
	//odomMapFile.open("/home/zuyuan/ros_ws/src/tutorial_pkg/odomMapData.csv",ios::out | ios::app);
	// keep spinning loop until user presses Ctrl+C
	while (ros::ok() && keepMoving){
	if (STAGE==0){
	    moveForward(mfs);
	   // displayMessages();
	}
		ros::spinOnce(); // Allow ROS to process incoming messages
		rate.sleep();   // or loop_rate.sleep();
	}
	//odomVelFile.close();
	//odomMapFile.close();
}

int main(int argc, char **argv)
{	// Initiate new ROS node named "stopper"
   ros::init(argc, argv, "stopper");
   
   // Create new stopper object
   
   Stopper stopper;
   beginTime = ros::Time::now().toSec(); // start counting time, for velocity figure
   ros::Duration(3).sleep(); // wait for 3 seconds for Gazebo to spawn
   // Start the movement
   stopper.startMoving();
	
   return 0;
}
