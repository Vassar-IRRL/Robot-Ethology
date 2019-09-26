/**
Vassar Cognitive Science - Robot Ethology

This program operates a kipr-link-based robot (equipped with analog photo, ir, contact sensors) based on a specified subsumption hierarchy.
The initial behavior at program execution uses the hard coded behaviors listed in the "subsumption_hierarchy" array.
Behaviors can be edited and altered by pressing the side button on the link and altered using the buttons on the screen.
Once set, the side button can be pressed again to return to execution mode.

To prevent students from knowing the boot behavior, the subsumption_hierarchy is randomized after the first button press.  However, this behavior can be restored by simply restarting the program.

Course:			211 - Perception & Action
Instructors:	Ken Livingston, Ryan Luke Johns
Author: 		RLJ <ryanjohns@vassar.edu> (GUI Version)
Date:			September 2019
Contributions:	Earlier version by Nick Livingston (2018)
*/

#include <stdlib.h> //import for min, max, etc.
#include <stdbool.h> //import for boolean support

//define integer keys for each action type
#define SEEK_LIGHT_TYPE 0
#define SEEK_DARK_TYPE 1
#define APPROACH_TYPE 2
#define AVOID_TYPE 3
#define ESCAPE_F_TYPE 4
#define ESCAPE_B_TYPE 5
#define CRUISE_S_TYPE 6
#define CRUISE_A_TYPE 7

//set pin address for hardware
#define RIGHT_IR_PIN 0 //sensors
#define LEFT_IR_PIN 1
#define RIGHT_PHOTO_PIN 2
#define LEFT_PHOTO_PIN 3
#define FRONT_BUMP_PIN 4
#define BACK_BUMP_PIN 5

#define RIGHT_MOTOR_PIN 0 //servos
#define LEFT_MOTOR_PIN 2

//here we define a new kind of variable type called "behavior" that contains properties for type (indexing definitions above), rank, and an active/inactive boolean
typedef struct behavior{
	const char *title;
	int type;
	int rank;
	bool is_active;
} behavior;

//this comparator function is used in the qsort function for sorting our behavior list.  Active things always go before inactive things, and if both are active then the are ordered by rank.
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

//GUI FUNCTIONS
void update_gui(); //this function contains our gui update feature
void print_subsumption_hierarchy(struct behavior *array, size_t len);
void print_set_hierarchy();
void randomize_hierarchy();

//*************************************************** Variable Definitions ****************************************************/


//global variables to store all current sensor values accessible to all functions and updated by the "read_sensors" function
int left_photo_value, right_photo_value, front_bump_value, back_bump_value, left_ir_value, right_ir_value;

//threshold values
int avoid_threshold   = 300; 	//the absolute difference between IR readings has to be above this for the avoid action
int approach_threshold = 300;	//the absolute difference between IR readings has to be below this for the approach action
int photo_threshold = 8;		//the absolute difference between photo sensor readings has to be above this for seek light/dark actions
float photo_max = 200.0; 		//approximate max possible photo reading (set from observation)

int timer_duration = 500;		//the time in milliseconds to wait between calling action commands.  This value is changed by each drive command called by actions
unsigned long start_time = 0;	//store the system time each time we start an action so we can see if our time has elapsed without a blocking delay

//this struct defines the initial behavior, but also describes all possible behaviors.  New ones could be added, or the order can be moved around.
//this behavior runs once at the beginning of the program until the gui is accessed.  There is no need to change the rank value manually, just change the order and set
//the ones you want to be active to "true".  The element at the top is at the top of the hierarchy.
struct behavior subsumption_hierarchy[] = {
	{"ESCAPE FRONT", ESCAPE_F_TYPE, 0, true},
	{"ESCAPE BACK", ESCAPE_B_TYPE, 0, true},
	{"AVOID", AVOID_TYPE, 0, true},
	{"SEEK LIGHT", SEEK_LIGHT_TYPE, 0, true},
	{"CRUISE STRAIGHT", CRUISE_S_TYPE, 0, true},
	{"SEEK DARK",  SEEK_DARK_TYPE, 0, false},
	{"APPROACH", APPROACH_TYPE, 0, false},
	{"CRUISE ARC", CRUISE_A_TYPE, 0, false}
};
int hierarchy_length; //set in main function based on number of elements in subsumption_hierarchy defined above
int cursor_row = 0; //the row that the cursor is on in gui mode
bool show_gui = false;	//boolean toggled by pushing the white side button on the kipr link
bool first_gui = true; 	//on first exposure to gui, we randomize the hierarchy so the initialized behavior can't be observed
bool is_side_update = false;			//sort on button press
bool update_operating_console = false;	//a boolean to tell us when to update the operating console.  If we constantly reprint and clear, we get flicker, so we only print once when necessary


//*************************************************** Function Definitions ****************************************************//
int main() 
{
	hierarchy_length = sizeof(subsumption_hierarchy) / sizeof(behavior); //set this variable once for loopin trhough the hierarchy
	//bump sensor values are always above 1000, we store a less-frequently updated version of the bumper in this var, so we need to give it an initial value that is high
	front_bump_value = 1000;
	back_bump_value = 1000;
	
	enable_servo(LEFT_MOTOR_PIN);	//initialize both motors and set speed to zero
	enable_servo(RIGHT_MOTOR_PIN);
	drive(0.0,0.0,1.0);
	
	while(true){ //this is an infinite loop (true is always true)
		update_gui(); //update our gui in any case
		
		if(!show_gui){ //if we aren't showing the gui, we must be sensing and acting
			
			if(update_operating_console){
				//only enable the servos once when returning from the gui menu, this boolean is disabled in the next print_set_hierarchy function
				enable_servo(LEFT_MOTOR_PIN);
				enable_servo(RIGHT_MOTOR_PIN);
				drive(0.0,0.0,2.0);
			}
			print_set_hierarchy(); //print the current subsumption hierarchy to the screen (only executes if gui has been accessed once before)
			
			read_sensors(); //read all sensors and set global variables of their readouts
			
			if(timer_elapsed()){ //any time a drive message is called, the timer is updated.  Until it is called again this should always return true
				bool execute_action = false; //tell us if we have executed ANY action
				size_t i;	//counter for hierarchy for loop
				for(i=0; i<hierarchy_length; i++){ //for each behavior in our hierarchy
					if(subsumption_hierarchy[i].is_active){ //if the behavior at this index is active...
						switch(subsumption_hierarchy[i].type){ //run a switch/case statement to see which type this behavior is and do the appropriate action
							//for the specified hierarchy type, check if we should execute the action, and do it if so.  If not, continue the for loop.  If so, execute action and break.
							case SEEK_LIGHT_TYPE:
							execute_action = is_above_photo_differential(photo_threshold);
							if(execute_action) seek_light();
							break;
							case SEEK_DARK_TYPE:
							execute_action = is_above_photo_differential(photo_threshold);
							if(execute_action) seek_dark();
							break;
							case APPROACH_TYPE:
							execute_action = is_above_distance_threshold(approach_threshold);
							if(execute_action) approach();
							break;
							case AVOID_TYPE:
							execute_action = is_above_distance_threshold(avoid_threshold);
							if(execute_action) avoid();
							break;
							case ESCAPE_F_TYPE:
							execute_action = is_front_bump();
							if(execute_action) escape_front();
							break;
							case ESCAPE_B_TYPE:
							execute_action = is_back_bump();
							if(execute_action) escape_back();
							break;
							case CRUISE_S_TYPE:
							execute_action = true;
							cruise_straight();
							break;
							case CRUISE_A_TYPE:
							execute_action = true;
							cruise_arc();		
							break;
						} //end hierarchy type switch
					} //end if active
					if(execute_action){
						break; //if any action was executed, break out of the subsumption hierarchy loop altogether
					}//end if execute action	
					else{
						stop(); //if there is no action, stop
					}
				} //end for each item in hierarchy loop
			}//end if timer elapsed
		}//end if not show gui
		
		else{
			disable_servos(); //disable all servo motors if we are in gui mode
		}
	}//end while true
	
	return 0; //due to infinite while loop, we will never get here
}
/******************************************************/
bool timer_elapsed(){
	return (systime() > (start_time + timer_duration)); //return true if the current time is greater than our start time plus timer duration
}
/******************************************************/
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

/******************************************************/
float map(float value, float start_range_low, float start_range_high, float target_range_low, float target_range_high){
	//value = min(max(value, start_range_low), start_range_high); //clip to start range bounds (doesn't work with inverted start range vals)
	return target_range_low + ((value - start_range_low)/(start_range_high - start_range_low)) * (target_range_high - target_range_low);
}

//===============================GUI RELATED CODE========================================
//===============================GUI RELATED CODE========================================
//===============================GUI RELATED CODE========================================
void update_gui(){
	if(side_button_clicked()){
		show_gui = !show_gui; //toggle our gui by pressing the side button
		is_side_update = show_gui; //boolean to do certain behaviors once at button press
		update_operating_console = true; //boolean to update the home console once
	}
	
	if(show_gui){
		if(first_gui){
			randomize_hierarchy(); //this only ever happens once per program
			first_gui = false;
		}
		
		//stop();
		bool cursor_update = false;
		bool hierarchy_update = false;
		
		set_extra_buttons_visible(1); //we turn off the extra buttons (buttons xyz) when we are not in showgui mode, so we need to activate them here
		
		set_a_button_text(subsumption_hierarchy[cursor_row].is_active?"Deactivate":"Activate"); //set text to display activate or deactivate based on the behavior the cursor is on
		set_b_button_text(subsumption_hierarchy[cursor_row].is_active?"Move Up":""); //set text to display "move up" or nothing based on the behavior the cursor is on
		set_y_button_text(subsumption_hierarchy[cursor_row].is_active?"Move Down":"");	//set text to display "move down" or nothing based on the behavior the cursor is on
		
		set_c_button_text("\u25B2"); //up triangle unicode
		set_z_button_text("\u25BC"); //unicode down triangle
		
		set_x_button_text("Reset");	//reset button for deactivating all
		
		if(c_button_clicked()){ //move the cursor up
			cursor_row = (cursor_row - 1); //up cursor
			if(cursor_row < 0) cursor_row += hierarchy_length; //if we go past zero, loop back to the end of the list
			cursor_row = cursor_row%hierarchy_length;
			cursor_update = true; //we've updated
		}
		
		else if(z_button_clicked()) { //move cursor down
			cursor_row = (cursor_row + 1)%hierarchy_length; //move cursor down and use modulus function to loop back to zero if we go down too far
			cursor_update = true;
		}
		
		else if(a_button_clicked()){ //activate or deactivate button
			subsumption_hierarchy[cursor_row].is_active = !subsumption_hierarchy[cursor_row].is_active; //toggle our active state
			hierarchy_update = true;
		}
		
		else if(b_button_clicked()){
			subsumption_hierarchy[cursor_row].rank -= 2; //move up
			hierarchy_update = true;
		}
		
		else if(y_button_clicked()){
			subsumption_hierarchy[cursor_row].rank += 2; //move down
			hierarchy_update = true;
		}
		
		else if(x_button_clicked()){ //reset all button
			size_t i; 
			for(i=0; i<hierarchy_length; i++){
				subsumption_hierarchy[i].is_active = false;
			}
			hierarchy_update = true;
		}
		
		if(cursor_update || is_side_update || hierarchy_update){ //if we pressed anything at all
			
			qsort(subsumption_hierarchy, hierarchy_length, sizeof(behavior), compare_ranks); //sort our hierarchy based on rank value
			
			size_t i;
			for(i=0; i<hierarchy_length; i++){
				if(subsumption_hierarchy[i].is_active) subsumption_hierarchy[i].rank = i; //now reset the index of each sorted active behavior to be sequential with a step size of one
				else subsumption_hierarchy[i].rank = hierarchy_length + 1; //give inactive behaviors a constant "poor" rank which is helpful to ensure new ones always jump above.
			}
			
			console_clear(); // clear the console
			print_subsumption_hierarchy(subsumption_hierarchy, hierarchy_length); //print the hierarchy and interface
			is_side_update = false; //turn off the is_side_update boolean so we don't get screen flicker until we update the cursor or hierarchy next
		}
		
	}
	else{
		set_a_button_text("");	//if we are not in show_gui mode, we must be operating, set our buttons to show nothing and hide the extra buttons
		set_b_button_text("");	
		set_c_button_text("");
		set_extra_buttons_visible(0);
	}
}
//-----------------RANDOMIZE HIERARCHY AND DEACTIVATE ALL-------------
void randomize_hierarchy(){
	size_t i;
	for(i=0; i<hierarchy_length; i++){
		subsumption_hierarchy[i].is_active = false;
		subsumption_hierarchy[i].rank = rand();
	}
	qsort(subsumption_hierarchy, hierarchy_length, sizeof(behavior), compare_ranks); //sort our hierarchy based on rank value
}
//-------------------------MANAGE SCREEN PRINTING OF GUI--------------------
void print_subsumption_hierarchy(struct behavior *array, size_t len){ 
	size_t i;
	for(i=0; i<len; i++){
		display_printf(1,i,"%s          ",array[i].title);
		if(array[i].is_active){
			display_printf(17,i,"Active  ");
		}
		else{
			display_printf(17,i,"Inactive ");
		}
		if(i == cursor_row){
			display_printf(0, i, ">");
			display_printf(25, i, "<");
		}
		else{
			display_printf(0, i, " ");
			display_printf(25, i, " ");
		}
		//display_printf(35, i, "%d", array[i].rank); //debug for showing rank
	}
}
//--------------------MANAGE SCREEN PRINTING WHEN OPERATING---------------------
void print_set_hierarchy(){ 
	if(update_operating_console && !first_gui){
		console_clear();
		size_t i;
		for(i=0; i<hierarchy_length; i++){
			if(subsumption_hierarchy[i].is_active) printf(" %s\n",subsumption_hierarchy[i].title);
		}
		update_operating_console = false; //this only happens once per button press if we are not showing gui
	}
}
//============================END GUI RELATED CODE========================================
