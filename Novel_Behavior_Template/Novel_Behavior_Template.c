/**
Vassar Cognitive Science - Robot Ethology

PROGRAM NAME:  ###Novel Behavior Template (REPLACE)###

TEAM NAME:	###NOVEL TEAM NAME###
STUDENTS:	###YOUR NAMES HERE###

COURSE:			211 - Perception & Action
INSTRUCTORS:	Ken Livingston, Ryan Luke Johns
TERM:			Fall 2019

PROGRAM DESCRIPTION:
###Describe what this program does in this area.  What is the overall novel behavior?  Can you condense the fundamental idea of the code into a brief overview here for your readers?###
###I.E.  This program operates a kipr-link-based robot (equipped with analog photo, ir, contact sensors)...###
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
//###ADD ANY OTHER SENSOR PIN ADDRESSES AND SHORTHAND NAMES HERE.

#define RIGHT_MOTOR_PIN 0 //servos
#define LEFT_MOTOR_PIN 2 
//###ADD ANY OTHER ACTUATOR NAMES AND PIN NUMBERS HERE


//*************************************************** Function Declarations ***********************************************************//
//###DECLARE ALL OF YOUR FUNCTIONS HERE.  WHAT DO THEY RETURN (THE VARIABLE TYPE ON THE LEFT) AND WHAT ARE THEIR INPUTS (THE VALUES IN THE PARENTHESES)###

//PERCEPTION FUNCTIONS
void read_sensors(); 	//function to read all sensor values and save to global vars
//###ADD PERCEPTION FUNCTIONS HERE###

//ACTION FUNCTIONS
int example_do_something(float foo); //a function with a float input that returns an integer
void drive(float left, float right, float delay_seconds); //drive with a certain motor speed for a number of seconds
//###ADD OTHER nDRIVING/GRIPPING/ACTION FUNCTIONS HERE###

//HELPER FUNCTIONS 
bool timer_elapsed();	//return true if our timer has elapsed
float map(float value, float start_range_low, float start_range_high, float target_range_low, float target_range_high); //remap a value from a source range to a new range
//###ADD ANY OTHER HELPER FUNCTIONS IN THIS AREA###


//*************************************************** Variable Definitions ****************************************************/
//### DEFINE GLOBAL VARIABLES HERE###
int example_integer_variable = 100; //this variable stores an example that is not used elsewhere

//global variables to store all current sensor values accessible to all functions and updated by the "read_sensors" function
int left_photo_value, right_photo_value, front_bump_value, back_bump_value, left_ir_value, right_ir_value;

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
	
	enable_servo(LEFT_MOTOR_PIN);	//initialize both motors
	enable_servo(RIGHT_MOTOR_PIN);
	drive(0.0,0.0,1.0);				//set our drive speed to zero so we aren't moving at the start
	
	while(true){ //this is an infinite loop (true is always true!)
		
		read_sensors(); //read all sensors and set global variables of their readouts
		
		if(timer_elapsed()){ //any time a drive message is called, the timer is updated.  Until it is called again this should always return true (i.e. stuff inside happens)
			
			int value = example_do_something(3.4);//do something random and useless (replace with useful and intended functions)
			
		}//end if timer elapsed
	}//end while true
	
	return 0; //due to infinite while loop, we will never get here
}

//================================================================================================================//
//====================================================PERCEPTION==================================================//
//================================================================================================================//
/******************************************************/
/**
This function is called once per loop, and reads all the sensor values and sets the appropriate global variables defined above.
**/
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

//================================================================================================================//
//========================================================ACTION==================================================//
//================================================================================================================//
/******************************************************/
/**
This is an example of a function, it checks if the input value is greater than three.  If it is, it returns the integer 3 and drives straight.

Inputs:
	[foo] A floating point number
  
Returns 3 if foo is greater than 3, otherwise it returns zero and does nothing
**/

int example_do_something(float foo){
	if(foo > 3.0){
		drive(1.0,1.0,0.25);
		return 3;
	}
	else return 0;
}
/******************************************************/
/**
The drive function takes a left and right motor speed and a delay time amount as inputs and triggers the wheels to drive.  It also resets the bumper value to "not bumped" each time it is called.

Inputs:
	[left] The left wheel speed, between -1.0 and 1.0
	[right] The right wheel speed, between -1.0 and 1.0
	[delay_seconds] The delay time in seconds (0 to MAX_FLOAT)
**/

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


//================================================================================================================//
//========================================================HELPERS=================================================//
//================================================================================================================//
/******************************************************/
/**
Checks if the timer has elapsed.

Returns true if the global variable start time + the timer duration is greater than the system clock time (in milliseconds)
**/
bool timer_elapsed(){
	return (systime() > (start_time + timer_duration)); //return true if the current time is greater than our start time plus timer duration
}

/******************************************************/
/**
Map a value from an input range to a new range.

Inputs:
	[value] the starting number to remap
	[start_range_low] the low value of the initial bounds
	[start_range_high] the high value of the initial bounds
	[target_range_low] the low value of the target bounds
	[target_range_high] the high value of the target bounds

Returns the remapped value as a float
**/
float map(float value, float start_range_low, float start_range_high, float target_range_low, float target_range_high){
	return target_range_low + ((value - start_range_low)/(start_range_high - start_range_low)) * (target_range_high - target_range_low);
	/******************************************************/
}
