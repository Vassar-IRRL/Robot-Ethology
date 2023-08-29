/*
Vassar Cognitive Science - Robot Ethology

This program operates a kipr-link-based robot (equipped with analog photo, ir, contact sensors) based on a specified subsumption hierarchy.
The initial behavior at program execution uses the hard coded behaviors listed in the "subsumption_hierarchy" array.
Behaviors can be edited and altered by pressing the side button on the link and altered using the buttons on the screen.
Once set, the side button can be pressed again to return to execution mode.

To prevent students from knowing the boot behavior, the subsumption_hierarchy is randomized after the first button press.  However, this behavior can be restored by simply restarting the program.

Course:			211 - Perception & Action
Instructors:	Ken Livingston
Author: 		RLJ <ryanjohns@vassar.edu> (GUI Version)
Date:			August 2023
Revisions:	    Earlier version (2018) and revisions (2023) by Nick Livingston
*/

// *** Import Libraries *** //

#include <kipr/wombat.h> // KIPR Wombat native library
#include <stdlib.h>	 // library for general purpose functions
#include <stdbool.h> // library for boolean support

// *** Define integer keys for each action type *** //
#define SEEK_LIGHT_TYPE 0
#define SEEK_DARK_TYPE 1
#define APPROACH_TYPE 2
#define AVOID_TYPE 3
#define ESCAPE_F_TYPE 4
#define ESCAPE_B_TYPE 5
#define CRUISE_S_TYPE 6
#define CRUISE_A_TYPE 7

// *** Define PIN Address *** //

#define RIGHT_IR_PIN 2
#define LEFT_IR_PIN 3
#define RIGHT_PHOTO_PIN 0
#define LEFT_PHOTO_PIN 1 // analog sensors (IRs, photos)

#define FRONT_BUMP_LEFT_PIN 5
#define FRONT_BUMP_CENTER_PIN 3
#define FRONT_BUMP_RIGHT_PIN 4
#define BACK_BUMP_LEFT_PIN 2
#define BACK_BUMP_CENTER_PIN 0
#define BACK_BUMP_RIGHT_PIN 1

#define RIGHT_MOTOR_PIN 0
#define LEFT_MOTOR_PIN 1 // servos

// *** Define a new kind of variable type called "behavior" that contains properties for type (indexing definitions above), rank, and an active/inactive boolean *** //
typedef struct behavior{
	const char *title;
	int type;
	int rank;
	bool is_active;
} behavior;

// *** Define a comparator function used in the qsort function for sorting our behavior list.  Active things always go before inactive things, and if both are active then the are ordered by rank. *** //
int compare_ranks(const void *a, const void *b)  
{ 	
	bool is_active_a = ((struct behavior *)a)->is_active; 
	bool is_active_b = ((struct behavior *)b)->is_active; 
	if(is_active_a != is_active_b){
		return is_active_b;
	}
	int rank_a = ((struct behavior *)a)->rank; 
	int rank_b = ((struct behavior *)b)->rank;  
	return (rank_a - rank_b); 
} 

//*************************************************** Function Declarations ***********************************************************//
// PERCEPTION FUNCTIONS
void read_sensors();							 // read all sensor values and save to global variables
bool is_above_distance_threshold(int threshold); // return true if one and only one IR sensor is above the specified threshold
bool is_above_photo_differential(int threshold); // return true if the absolute difference between photo sensor values is above the specified threshold
bool is_front_bump();							 // return true if one of the front bumpers was hit
bool is_back_bump();							 // return true if one of the back bumpers was hit
bool timer_elapsed();							 // return true if our timer has elapsed

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

// BUILT-IN FUNCTIONS
void enable_servo(int pin);						// enable servo at the specified pin
int analog_et(int pin);							// get the 10-bit analog value of a sensor on the specified pin
int digital(int pin);							// get the digital value of a sensor on the specified pin
unsigned long systime();						// get the system time
void set_servo_position(int pin, int position); // set a servo at the specified pin to the specified position

// *** Variable Definitions *** //

// global variables to store all current sensor values accessible to all functions and updated by the "read_sensors" function
int right_photo_value, left_photo_value, right_ir_value, left_ir_value, front_bump_left_value, front_bump_center_value, front_bump_right_value, back_bump_left_value, back_bump_center_value, back_bump_right_value;

// threshold values
int avoid_threshold = 1600;	   // the absolute difference between IR readings has to be above this for the avoid action
int approach_threshold = 1600; // the absolute difference between IR readings has to be below this for the approach action
int photo_threshold = 200;	   // the absolute difference between photo sensor readings has to be above this for seek light/dark actions

// timer
int timer_duration = 500;	  // the time in milliseconds to wait between calling action commands, changed by each drive command called by actions
unsigned long start_time = 0; // store the system time each time we start an action so we can see if our time has elapsed without a blocking delay

// *** Function Definitions *** //

//this struct defines the initial behavior, but also describes all possible behaviors.  New ones could be added, or the order can be moved around.
//this behavior runs once at the beginning of the program until the gui is accessed.  There is no need to change the rank value manually, just change the order and set
//the ones you want to be active to "true".  The element at the top is at the top of the hierarchy.
struct behavior subsumption_hierarchy[] = {
	{"ESCAPE FRONT", ESCAPE_F_TYPE, 0, false},
	{"ESCAPE BACK", ESCAPE_B_TYPE, 0, false},
	{"AVOID", AVOID_TYPE, 0, true},
	{"SEEK LIGHT", SEEK_LIGHT_TYPE, 0, false},
	{"CRUISE STRAIGHT", CRUISE_S_TYPE, 0, true},
	{"SEEK DARK",  SEEK_DARK_TYPE, 0, false},
	{"APPROACH", APPROACH_TYPE, 0, false},
	{"CRUISE ARC", CRUISE_A_TYPE, 0, false}
};
int hierarchy_length; //set in main function based on number of elements in subsumption_hierarchy defined above

//==================================//
//===============MAIN===============//
//==================================//

int main() 
{
	hierarchy_length = sizeof(subsumption_hierarchy) / sizeof(behavior); //set this variable once for loopin trhough the hierarchy
	
	enable_servo(LEFT_MOTOR_PIN);	//initialize both motors and set speed to zero
	enable_servo(RIGHT_MOTOR_PIN);
	drive(0.0,0.0,1.0);
	
	while(true){ //this is an infinite loop (true is always true)
		read_sensors(); //read all sensors and set global variables of their readouts
		if(is_front_bump())
        {
            escape_front();
        }
        else if(is_above_distance_threshold(avoid_threshold))
        {
            avoid();
        }
        else
        {
			cruise_straight();
        }
    }//end while true
	
	return 0; //due to infinite while loop, we will never get here
}

//========================================//
//===============PERCEPTION===============//
//========================================//

void read_sensors()
{
	right_photo_value = analog_et(RIGHT_PHOTO_PIN);			// read the photo sensor at RIGHT_PHOTO_PIN; *** NOTE: greater value means less light ***
	left_photo_value = analog_et(LEFT_PHOTO_PIN);			// read the photo sensor at LEFT_PHOTO_PIN; *** NOTE: greater value means less light ***
	right_ir_value = analog_et(RIGHT_IR_PIN);				// read the IR sensor at RIGHT_IR_PIN
	left_ir_value = analog_et(LEFT_IR_PIN);					// read the IR sensor at LEFT_IR_PIN
	// read the bumpers
	front_bump_left_value = digital(FRONT_BUMP_LEFT_PIN);   // read the bumper at FRONT_BUMP_LEFT_PIN
	front_bump_center_value = digital(FRONT_BUMP_CENTER_PIN); // read the bumper at FRONT_BUMP_CENTER_PIN
	front_bump_right_value = digital(FRONT_BUMP_RIGHT_PIN);  // read the bumper at FRONT_BUMP_RIGHT_PIN
	back_bump_left_value = digital(BACK_BUMP_LEFT_PIN);	// read the bumper at BACK_BUMP_LEFT_PIN
	back_bump_center_value = digital(BACK_BUMP_CENTER_PIN);  // read the bumper at BACK_BUMP_CENTER_PIN
	back_bump_right_value = digital(BACK_BUMP_RIGHT_PIN);	// read the bumper at BACK_BUMP_RIGHT_PIN	
}
/******************************************************/
bool is_above_photo_differential(int threshold)
{
	int photo_difference = abs(right_photo_value - left_photo_value); // get the difference between the photo values
	return photo_difference > threshold;							  // returns true if the absolute difference between photo sensors is greater than the threshold, otherwise false
}
/******************************************************/
bool is_above_distance_threshold(int threshold)
{
	return (left_ir_value > threshold || right_ir_value > threshold) && !(left_ir_value > threshold && right_ir_value > threshold);
	// returns true if one (exclusive) IR value is above the threshold, otherwise false
}

/******************************************************/
bool is_front_bump()
{
	return (front_bump_left_value == 1 || front_bump_center_value == 1 || front_bump_right_value == 1); // return true if one of the front bump values is 1, otherwise false
}
/******************************************************/
bool is_back_bump()
{
	return (back_bump_left_value == 1 || back_bump_center_value == 1 || back_bump_right_value == 1); // return true if one of the back bump values is 1, otherwise false
}
/******************************************************/

//====================================//
//===============ACTION===============//
//====================================//

/******************************************************/
void drive(float left, float right, float delay_seconds)
{
	// 850 is full motor speed clockwise, 1250 is full motor speed counterclockwise
	// Servo is stopped from ~1044 to 1055

	float left_speed = map(left, -1.0, 1.0, 0, 2047); // call the map function to map our speed (set between -1 and 1) to the appropriate range of motor values
	float right_speed = map(right, -1.0, 1.0, 2047, 0);

	timer_duration = (int)(delay_seconds * 1000.0); // multiply our desired time in seconds by 1000 to get milliseconds and update this global variable
	start_time = systime();							// update our start time to reflect the time we start driving (in ms)

	set_servo_position(LEFT_MOTOR_PIN, left_speed);
	set_servo_position(RIGHT_MOTOR_PIN, right_speed); // set the servos to run at the mapped speed
}
/******************************************************/
void cruise_straight()
{
	drive(0.50, 0.50, 0.25);
}
/******************************************************/
void cruise_arc()
{
	drive(0.25, 0.4, 0.5);
}
/******************************************************/
void stop()
{
	drive(0.0, 0.0, 0.25);
}
/******************************************************/
void escape_front()
{
	
    if(front_bump_left_value == 1 || front_bump_center_value == 1)
    {
        drive(-0.08, -1, 2.5); //drive backwards in an arc
    }
    else
    {
        drive(-1, -0.08, 2.5); //drive backwards in an arc
    }
}
/******************************************************/
void escape_back()
{
	drive(0.9, 0.9, 0.25); //drive forward a little
}
/******************************************************/
void seek_light()
{
	// greater photo_value means less light
	int photo_difference = right_photo_value - left_photo_value;
	// positive photo_difference means left sensor is brighter
	if (photo_difference > 0){
		drive(-0.2, 0.2, 0.25);
	}
	// negative photo_difference means right sensor is brighter
	if (photo_difference < 0){
		drive(0.2, -0.2, 0.25);
	}
}
/******************************************************/
void seek_dark()
{
	// greater photo_value means less light
	int photo_difference = right_photo_value - left_photo_value;
	// positive photo_difference means left sensor is brighter
	if (photo_difference > 0){
		drive(-0.2, 0.2, 0.25);
	}
	// negative photo_difference means right sensor is brighter
	if (photo_difference < 0){
		drive(0.2, -0.2, 0.25);
	}
}
/******************************************************/
void avoid()
{
	if (left_ir_value > avoid_threshold)
	{
		drive(0.5, -0.5, 0.1);
	}

	else if (right_ir_value > avoid_threshold)
	{
		drive(-0.5, 0.5, 0.1);
	}
}
/******************************************************/
void approach()
{
	if (left_ir_value > approach_threshold)
	{
		drive(0.1, 0.9, 0.5);
	}
	else if (right_ir_value > approach_threshold)
	{
		drive(0.9, 0.1, 0.5);
	}
}

//=====================================//
//===============HELPERS===============//
//=====================================//

bool timer_elapsed()
{
	return (systime() > (start_time + timer_duration)); // return true if the current time is greater than our start time plus timer duration
}
/******************************************************/
float map(float value, float start_range_low, float start_range_high, float target_range_low, float target_range_high)
{
	return target_range_low + ((value - start_range_low) / (start_range_high - start_range_low)) * (target_range_high - target_range_low);
	// remap a value from a source range to a new range
}
