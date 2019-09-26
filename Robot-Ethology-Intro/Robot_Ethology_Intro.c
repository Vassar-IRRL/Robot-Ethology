/**
Vassar Cognitive Science - Robot Ethology

This program operates a kipr-link-based robot (equipped with analog photo, ir, contact sensors) with a specified subsumption hierarchy.

Course:			211 - Perception & Action
Instructors:	Ken Livingston, Ryan Luke Johns
Authors: 		N. Livingston, RLJ <ryanjohns@vassar.edu> (GUI Version)
Date:			September 2019
*/

#include <stdlib.h> //import for min, max, etc.
#include <stdbool.h> //import for boolean support

//set pin address for hardware
#define RIGHT_IR_PIN 0 //sensors
#define LEFT_IR_PIN 1
#define RIGHT_PHOTO_PIN 2
#define LEFT_PHOTO_PIN 3
#define FRONT_BUMP_PIN 4
#define BACK_BUMP_PIN 5

#define RIGHT_MOTOR_PIN 0 //servos
#define LEFT_MOTOR_PIN 2 

//*************************************************** Function Declarations ***********************************************************//
//CHECKS
void read_sensors(); //function to read all sensor values and save to global vars
bool is_above_distance_threshold(int threshold); //return true if one and only one IR sensor is above the specified threshold
bool is_above_photo_differential(int threshold); //return true if the absolute difference between photo sensor values is above the specified threshold
bool is_front_bump();	//return true if the front bumper was hit
bool is_back_bump();	//return true if the back bumper was hit
bool timer_elapsed();	//return true if our timer has elapsed

//ACTIONS
void escape_front();
void escape_back();
void seek_light();
void seek_dark();
void avoid();
void approach();
void cruise_straight();
void cruise_arc();
void stop();

//MOTOR CONTROL
void drive(float left, float right, float delay_seconds); //drive with a certain motor speed for a number of seconds

//HELPER FUNCTIONS
float map(float value, float start_range_low, float start_range_high, float target_range_low, float target_range_high); //remap a value from a source range to a new range

//*************************************************** Variable Definitions ****************************************************/

//global variables to store all current sensor values accessible to all functions and updated by the "read_sensors" function
int left_photo_value, right_photo_value, front_bump_value, back_bump_value, left_ir_value, right_ir_value;

//threshold values
int avoid_threshold   = 300; 	//the absolute difference between IR readings has to be above this for the avoid action
int approach_threshold = 300;	//the absolute difference between IR readings has to be below this for the approach action
int photo_threshold = 8;		//the absolute difference between photo sensor readings has to be above this for seek light/dark actions
float photo_max = 200.0; 		//approximate max possible photo reading (set from observation)

//timer
int timer_duration = 500;		//the time in milliseconds to wait between calling action commands.  This value is changed by each drive command called by actions
unsigned long start_time = 0;	//store the system time each time we start an action so we can see if our time has elapsed without a blocking delay

//*************************************************** Function Definitions ****************************************************//

//================================================================================================================//
//=======================================================MAIN=====================================================//
//================================================================================================================//
int main() 
{
	//bump sensor values are always above 1000, we store a less-frequently updated version of the bumper in this var, so we need to give it an initial value that is high
	front_bump_value = 1000;
	back_bump_value = 1000;
	
	enable_servo(LEFT_MOTOR_PIN);	//initialize both motors and set speed to zero
	enable_servo(RIGHT_MOTOR_PIN);
	drive(0.0,0.0,1.0);
	
	while(true){ //this is an infinite loop (true is always true!)
		
		read_sensors(); //read all sensors and set global variables of their readouts
		
		if(timer_elapsed()){ //any time a drive message is called, the timer is updated.  Until it is called again this should always return true
			
			//subsumption hierarchy:  front, back, avoid, seek light, cruise straight
			if(is_front_bump()){
				escape_front();
			}
			else if(is_back_bump()){
				escape_back();
			}
			else if(is_above_distance_threshold(avoid_threshold)){
				avoid();
			}
			else if(is_above_photo_differential(photo_threshold)){
				seek_light();
			}
			else{
				cruise_straight();
			}
		}//end if timer elapsed
	}//end while true
	
	return 0; //due to infinite while loop, we will never get here
}

//================================================================================================================//
//====================================================PERCEPTION==================================================//
//================================================================================================================//
void read_sensors(){
	left_photo_value = analog_et(LEFT_PHOTO_PIN); 		//read the photo sensor at the LEFT_PHOTO_PIN
	right_photo_value = analog_et(RIGHT_PHOTO_PIN);		//read the photo sensor at the RIGHT_PHOTO_PIN
	left_ir_value = analog_et(LEFT_IR_PIN); 			//read the sensor for the left IR at LEFT_IR_PIN
	right_ir_value = analog_et(RIGHT_IR_PIN); 			//read the sensor for the right IR at RIGHT_IR_PIN
	
	int front_bump_sensor_value = analog10(FRONT_BUMP_PIN); //read the sensor at front_bump_pin
	int back_bump_sensor_value = analog10(BACK_BUMP_PIN);	//read the back bumper sensor
	//the default (unbumped)value of these sensors jumps between 900 and 1024, with contact reading values between 0 and 400.  We only care about these.
	//update this value whenever it is below a threshold (note this is sticky and we'll need to refresh it elsewhere)
	if(front_bump_sensor_value < 400){
		front_bump_value = front_bump_sensor_value; 
	}	
	if(back_bump_sensor_value < 400){
		back_bump_value = back_bump_sensor_value;
	}
}	

/******************************************************/
bool is_above_photo_differential(int threshold){
	int photo_difference = left_photo_value - right_photo_value;	//get the difference between the photo values
	return (abs(photo_difference) > threshold); 					//returns true if the absolute difference between photo sensors is greater than the threshold, otherwise false
}
/******************************************************/
bool is_above_distance_threshold(int threshold){
	return (left_ir_value > threshold || right_ir_value > threshold);  //returns true if one (exclusive) ir value is above the threshold, otherwise returns false
}
/******************************************************/
bool is_front_bump(){
	return (front_bump_value <= 400);	 //return true if our bump value is less than or equal to 400
}
/******************************************************/
bool is_back_bump(){
	return (back_bump_value <= 400);  //return true if our bump value is less than or equal to 400
}


//================================================================================================================//
//========================================================ACTION==================================================//
//================================================================================================================//
void drive(float left, float right, float delay_seconds){
	//850 is full motor speed clockwise, 1050 is stopped,  1250 is full motor speed counterclockwise
	//servo is stopped from ~1044 to 1055
	float left_speed = map(left, -1.0, 1.0, 850.0, 1250.0); //call the map function to map our speed (set between -1 and 1) to the appropriate range of motor values
	float right_speed = map(right, -1.0, 1.0, 1250.0, 850.0);
	
	timer_duration = (int)(delay_seconds * 1000.0); //multiply our desired time in seconds by 1000 to get milliseconds and update this global variable
	start_time = systime(); //update our start time to reflect the time we start driving (in ms)
	front_bump_value = 1000; //reset our sticky bumper values so we start this cycle as if we have not hit anything (hits read out <400)
	back_bump_value = 1000; 
	
	set_servo_position(LEFT_MOTOR_PIN, left_speed); //set the servos to run at the mapped speed
	set_servo_position(RIGHT_MOTOR_PIN, right_speed);
}
/******************************************************/
void cruise_straight(){
	drive(0.50, 0.50, 0.5);
}
/******************************************************/
void cruise_arc(){
	drive(0.25, 0.4, 0.5);
}
/******************************************************/
void stop(){
	drive(0.0, 0.0, 0.25);
}
/******************************************************/
void escape_front(){
	float bump_midpoint = 250.0;
	float bump_max = 400.0;
	
	if(front_bump_value < bump_midpoint){
		float backup_time = map((float)front_bump_value, 0.0, bump_midpoint, 0.3, 0.75); //spend more time backing up with an arc as we hit closer to the center
		drive(-0.1, -0.90, backup_time);
	}
	else if(front_bump_value >= bump_midpoint){
		float backup_time = map((float)front_bump_value, bump_midpoint, bump_max, 0.75, 0.3); //spend more time backing up with an arc as we hit closer to the center
		drive(-0.90, -0.0, backup_time);
	}
}
/******************************************************/
void escape_back(){
	float bump_midpoint = 250.0;
	float bump_max = 400.0;
	
	if(back_bump_value < bump_midpoint){
		float backup_time = map((float)back_bump_value, 0.0, bump_midpoint, 0.3, 0.75); //spend more time backing up with an arc as we hit closer to the center
		drive(0.9, 0.10, backup_time);
	}
	else if(back_bump_value >= bump_midpoint){
		float backup_time = map((float)back_bump_value, bump_midpoint, bump_max, 0.75, 0.3); //spend more time backing up with an arc as we hit closer to the center
		drive(0.10, 0.9, backup_time);
	}
}
/******************************************************/
void seek_light(){
	float left_servo;
	float right_servo;
	int photo_difference = left_photo_value - right_photo_value;
	if(abs(photo_difference) > photo_threshold){
		//if(photo_difference > photo_max) photo_difference = (int)photo_max;
		int multiplier = (photo_difference > 0)? 1:-1;
		right_servo = 0.2*multiplier;
		left_servo  = -right_servo;
	}
	drive(left_servo, right_servo, 0.25);
}
/******************************************************/
void seek_dark(){
	float left_servo;
	float right_servo;
	int photo_difference = left_photo_value - right_photo_value;
	if(abs(photo_difference) > photo_threshold){
		//if(photo_difference > photo_max) photo_difference = (int)photo_max;
		int multiplier = (photo_difference > 1)? -1:1;
		right_servo  = 0.2*multiplier;//(float)photo_difference/photo_max;
		left_servo = -right_servo;
	}
	drive(left_servo, right_servo, 0.25);
}
/******************************************************/
void avoid(){
	if(left_ir_value > avoid_threshold){
		drive(0.5, -0.5, 0.1);
	}
	
	else if(right_ir_value > avoid_threshold){
		drive(-0.5, 0.5, 0.1);
	}
}
/******************************************************/
void approach(){
	if(left_ir_value > approach_threshold){
		drive(0.1, 0.9, 0.5);
	}
	else if(right_ir_value > approach_threshold){
		drive(0.9, 0.1, 0.5);
	}
}

//================================================================================================================//
//========================================================HELPERS=================================================//
//================================================================================================================//
bool timer_elapsed(){
	return (systime() > (start_time + timer_duration)); //return true if the current time is greater than our start time plus timer duration
}
/******************************************************/
float map(float value, float start_range_low, float start_range_high, float target_range_low, float target_range_high){
	return target_range_low + ((value - start_range_low)/(start_range_high - start_range_low)) * (target_range_high - target_range_low);
}
