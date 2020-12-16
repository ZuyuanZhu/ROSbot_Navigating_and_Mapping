/*
  We will create a node called stopper that will make 
  the robot move forward until it detects an obstacle in front of it.

  We will use the laser sensor data to achieve this. 
  
  Laser data is published to the topic /scan.
*/

#include "../include/Stopper.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper(){
	keepMoving = true;
	//Advertise a new publisher for the simulated robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10); //10
	
	//Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("scan",1,&Stopper::scanCallback, this);
	//poseSub = node.subscribe("odom", );
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
	commandPub.publish(msg);
}

void Stopper::keepMiddleTorward2ndGap(unsigned int rightClosestRangeIndex, unsigned int leftClosestRangeIndex){
	int middleIndex;
	middleIndex = ceil((720-rightClosestRangeIndex - leftClosestRangeIndex)/2.0);
	ROS_INFO_STREAM("middleIndex " << middleIndex);
	if(middleIndex > 20){
		moveRight(TURN_RIGHT_SPEED_LOW);
		ROS_INFO_STREAM("Move right a little ");
	}
	else if((middleIndex < -10)){
		//moveLeft(TURN_LEFT_SPEED_LOW);
		ROS_INFO_STREAM("Move left a little ");
	}
	else{
		moveForward(FORWARD_SPEED_LOW/2.0);
		ROS_INFO_STREAM("Move forward a little ");
	}
		
}

void Stopper::keepMoveAlongWall(double range2Wall){
	ROS_INFO_STREAM("leftRange to the Wall: " << leftRange);
	if(leftRange > (range2Wall+0.2)){
		moveLeft(TURN_LEFT_SPEED_HIGH);
		ROS_INFO_STREAM("Turn left a little ");
		displayMessages();			
	} 
	if(leftRange < (range2Wall-0.2)){
		moveRight(TURN_RIGHT_SPEED_MIDDLE);
		ROS_INFO_STREAM("Turn right a little ");
		displayMessages();
	}
	else{
		moveForward(FORWARD_SPEED_LOW);	
		ROS_INFO_STREAM("Move forward a little ");	
		displayMessages();
	}
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
}


//process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	//	Find the closest range between the defined minimum and maximum angles
	ros::Rate rate(20); //20
	
	/*
	int	minIndex = ceil((LEFT_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);  // the transfermation is not right
	int	maxIndex = floor((RIGHT_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int frontIndex = floor((0 - scan->angle_min) / scan->angle_increment);		
	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++){
		if (scan->ranges[currIndex] < closestRange){
			closestRange = scan->ranges[currIndex];
		}
	}
	*/
	
	frontRange = scan->ranges[0];
	rightRange = scan->ranges[539];
	leftRange = scan->ranges[180];
	
	/*
	********************************************
	********** Put your code here **************
	********************************************
	*/
	
	// STAGE = 0, Initial stage, start moving towards the first gap before turing right
	if (((rightRange < 0.375)||(frontRange < 1.0)) && STAGE==0){
		ROS_INFO("Turn Right");
		moveRight(TURN_RIGHT_SPEED_HIGH);
		displayMessages();
	}
	// finished first turning
	
	//If the robot is in the middle of the first gate, keep moving forward
	if( (0.2 <leftRange) && (leftRange < 0.64) &&  (0.2 < rightRange) && (rightRange < 0.35) && (1.8< frontRange) && (frontRange < 1.9)){
		STAGE = 1;
		moveForward(FORWARD_SPEED_LOW);
		displayMessages();
	}
	
	if( (frontRange > 1.3) && (STAGE ==1)){  // Move through 1st gap
		moveForward(FORWARD_SPEED_LOW);
		displayMessages();
	}
	if((frontRange < 1.3)&&(STAGE ==1)){ //Moving forward ends
		STAGE =2;
		displayMessages();	
	}	
	if((frontRange < 1.3) && (STAGE ==2)){  //turn right, torwarding the 2nd gap
		moveRight(TURN_RIGHT_SPEED_LOW);
		displayMessages();
		if((((rightRange > 2.1)||(rightRange < 0.65))&&(frontRange > 0.89)&&(leftRange > 0.56))||((rightRange > 1.7) && (leftRange > 0.54) && (frontRange > 1.15))){ //laser noise makes it hard to find the right conditions
			STAGE = 3;				
		}
	}
	if((STAGE==3)){
		moveForward(FORWARD_SPEED_LOW);			
		/*
		********************************************
		****** find the closest range index ********
		*************** right side *****************
		********************************************
		*/
		int indexRightRangeMin = 600;
		int indexRightRangeMax = 719;
		float closestRightRange = scan->ranges[indexRightRangeMin];
		int closestRightRangeIndex = indexRightRangeMin;
		for (int currRightIndex = indexRightRangeMin + 1; currRightIndex <= indexRightRangeMax; currRightIndex++){
			if (scan->ranges[currRightIndex] < closestRightRange){
				closestRightRange = scan->ranges[currRightIndex];
				closestRightRangeIndex = currRightIndex;
			}
		}
		ROS_INFO_STREAM("closestRightRangeIndex: " << closestRightRangeIndex <<" "<< closestRightRange);		
		/*
		********************************************
		****** find the closest range index ********
		*************** left  side *****************
		********************************************
		*/
		int indexLeftRangeMin = 0;
		int indexLeftRangeMax = 150;
		float closestLeftRange = scan->ranges[indexLeftRangeMin];
		int closestLeftRangeIndex = indexLeftRangeMin;
		for (int currLeftIndex = indexLeftRangeMin + 1; currLeftIndex <= indexLeftRangeMax; currLeftIndex++){
			if (scan->ranges[currLeftIndex] < closestLeftRange){
				closestLeftRange = scan->ranges[currLeftIndex];
				closestLeftRangeIndex = currLeftIndex;
			}
		}
		ROS_INFO_STREAM("closestLeftRangeIndex: " << closestLeftRangeIndex <<" "<< closestLeftRange);		
		
		// Adjust heading angle torwards the 2nd gap
		if ((rightRange > 0.25) && (leftRange > 0.25)){  // make sure the robot won't adjust head angle when nearby the pillars
			keepMiddleTorward2ndGap(closestRightRangeIndex,closestLeftRangeIndex);
		}		
		displayMessages();
		
		// keep straight going through the 2nd gap
		if((closestLeftRange < 0.22)&& (closestRightRange < 0.22)){
			STAGE = 4;			// go through 2nd gap		
			displayMessages();				
		}
	}
	if((rightRange < 0.5 ) && (frontRange > 0.5) && (STAGE==4)){
		moveForward(FORWARD_SPEED_LOW);			//go through 2nd gap		
		displayMessages();			
	}
	else if((frontRange < 0.5)&& (STAGE==4)){ 
		STAGE = 5;	//passed 2nd gap, prepare to turn right	 		
	}
	if(((rightRange > 1.2)&& (rightRange < 2.2)) && (STAGE == 5)){
		moveRight(TURN_RIGHT_SPEED_MIDDLE); // turn right
		displayMessages();	
		if((frontRange > 0.7) && (leftRange < 0.341) && ((rightRange > 2.0)||(rightRange < 0.55))){
			STAGE = 6; // stop turning right, prepare to move forward along wall bottom
		}	
	}			
	if( (frontRange > 0.32) && (STAGE == 6)){
		moveForward(FORWARD_SPEED_LOW);
		double leftRange2Wall = 0.32;
		keepMoveAlongWall(leftRange2Wall); // move forward, parallel to the left wall	
		displayMessages();
	}
	else if( (frontRange < 0.32) && (STAGE == 6)){
		STAGE = 7;  // turn right, parallel to the wall (0,0) - (2.5, 0)
		displayMessages(); 				
	}
	if((frontRange < 2.2)&& (STAGE == 7)){  // turn right, parallel to the wall (0,0) - (2.5, 0)		
		moveRight(TURN_RIGHT_SPEED_MIDDLE);
		displayMessages(); 
	    if((leftRange < 0.35) && (leftRange > 0.28)  && (frontRange > 1.7) && (frontRange < 2.2) && ((rightRange < 1.0)||(rightRange > 2.0))){
	    	STAGE = 8;  // Stop turning right, prepare to move forward
			displayMessages();  	
	    } 	
	}
	if (STAGE == 8){
		moveForward(FORWARD_SPEED_HIGH);
		double leftRange2Wall2Chagrer = 0.32;
		keepMoveAlongWall(leftRange2Wall2Chagrer); // move forward, parallel to the left wall
		if((frontRange < 1.2) && (frontRange > 1.1)){
			STAGE = 9;
			moveStop();
			displayMessages(); 
			ROS_INFO_STREAM("ARRIVING AT CHARGER STATION");					
		}			
	}						
}

/*
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}
*/
void Stopper::startMoving(){
	ros::Rate rate(20); //20
	ROS_INFO("Start moving");
	
	// keep spinning loop until user presses Ctrl+C
	while (ros::ok() && keepMoving){
		if(STAGE ==0){  // Comment before public to student
			moveForward(FORWARD_SPEED_LOW);
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
