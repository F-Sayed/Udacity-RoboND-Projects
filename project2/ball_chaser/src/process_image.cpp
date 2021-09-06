#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
	ROS_INFO_STREAM("Setting robot velocities");

    // TODO: Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget srv;

	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	int left_count = 0;
	int right_count = 0;
	int front_count = 0;
	int img_size = img.height * img.step;

	for (int i=0; i<img_size; i+=3)
	{
		if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel)
		{
			float pos = i % img.step;

			if (pos < img.step * 0.33)
				left_count++;
			else if (pos > img.step * 0.67)
				right_count++;
			else 
				front_count++;
		}
		
		if (left_count + right_count + front_count > 200)
			break;
	}
	
	ROS_INFO_STREAM("Left: " << left_count << " Right: " << right_count << " Front: " << front_count << "\n");
	//personal touch: if there is no ball, slowly rotate in place until a ball is found	
	if (left_count == 0 && right_count == 0 && front_count == 0)
		drive_robot(0, 0.1);
	else if (left_count > right_count && left_count > front_count)
		drive_robot(0.3, -0.5);
	else if (right_count > front_count && right_count > left_count)
		drive_robot(0.3, 0.5);
	else if (front_count > left_count && front_count > right_count)
		drive_robot(0.5, 0);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
