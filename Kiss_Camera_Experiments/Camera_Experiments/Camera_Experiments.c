#include <stdlib.h> //import for min, max, etc.
#include <string.h>
#include <math.h>

const char *curr_img; //our always updating camera image
int prev_img_pixels[921600] = {}; //hard coded max dimensions of our saved image

int frame_difference(const char *img_a, int img_b[]); //function to show the difference between two images

int main()
{
	
	camera_open();
	camera_update();
	
	printf("Camera Dimensions: %d  x %d\n", get_camera_width(), get_camera_height()); //print the dimensions of our camera just for reference
	graphics_open(get_camera_width(), get_camera_height()); //this opens the screen up for drawing on, and sets the dimensions to the same as the camera
	
	curr_img = get_camera_frame(); //get the current camera frame and save it to curr_img
	const int rgb_count = get_camera_width()*get_camera_height()*3;
	//int prev_img_pixels[rgb_count] = {}; //make a new empty char array to hold previous images that has the same length as curr_img

	while(!get_key_state('Q'))
	{//if we have a keyboard, we can quit with the letter q, otherwise this loops perpetually
		camera_update(); //update the camera
		
		for(int y=0;y<get_camera_height();y++) {
			for(int x=0;x<get_camera_width();x++) {
			int pixel_index= 3*(get_camera_width()*y + x); //index of pixel at row Y, column X
			//pixel values are stored in BRG order
			prev_img_pixels[pixel_index] = curr_img[pixel_index + 0]; //save the current image pixel into our saved pixel array
			prev_img_pixels[pixel_index + 1] = curr_img[pixel_index + 1];
			prev_img_pixels[pixel_index + 2] = curr_img[pixel_index + 2];
			}
		}
		
		curr_img = get_camera_frame(); //get the new curr_img
		
		graphics_blit_enc(get_camera_frame(), BGR, 0, 0, get_camera_width(), get_camera_height()); //send our normal camera image to the graphics drawer
		
		frame_difference( curr_img, prev_img_pixels); //get our frame difference if we have more than one frame
		
		graphics_update();
}
	
	camera_close();
	graphics_close();
	
	return 0;
}

int frame_difference(const char *img_a, int img_b[]){ //a function to get the difference between two images
	int num_counted = 0; //store how many we have counted
	int total_difference = 0; //store the total difference for later taking an average
	for(int y=0;y<get_camera_height();y++) {
		for(int x=0;x<get_camera_width();x++) {
			int pixel_index= 3*(get_camera_width()*y + x); // index of pixel to paint into row r, column c
			//pixel values are stored in BRG order
			int a_pixel_blue = img_a[pixel_index + 0];
			int a_pixel_green = img_a[pixel_index + 1];
			int a_pixel_red = img_a[pixel_index + 2];
			
			int b_pixel_blue = img_b[pixel_index + 0];
			int b_pixel_green = img_b[pixel_index + 1];
			int b_pixel_red = img_b[pixel_index + 2];
			
			float a_brightness = (float)(a_pixel_blue + a_pixel_green + a_pixel_red)/3.0;
			float b_brightness = (float)(b_pixel_blue + b_pixel_green + b_pixel_red)/3.0;
			
			//if(x == 0 && y == 0){
			//	printf("a: %d, b: %d\n", a_pixel_blue, b_pixel_blue);
			//}
			
			int blue_diff = fabs(b_pixel_blue - a_pixel_blue);
			int green_diff = fabs(b_pixel_green - a_pixel_green);
			int red_diff = fabs(b_pixel_red - a_pixel_red);

			graphics_pixel(x,y,red_diff,green_diff, blue_diff);
			
			total_difference += blue_diff + green_diff + red_diff;
			num_counted++;
		}
	}
	
	int average_difference = total_difference / num_counted;
	return average_difference;
}
