#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"


#include <ctime>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;


geometry_msgs::Twist follow_cmd;
int world_state;
int lift = 0;
int direction = 0;
int last_direction = 0;
int got_lost_counter = 0;
int back_n_forth_counter = 0;
int good_counter = 0;
int i = 0;
int j = 0;

double last_linear_cmd = 0.0;

//Bumper Variable
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0, back_n_forth = 0, got_lost = 0, timer_on = 0;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCallback(const kobuki_msgs::BumperEvent msg){

    if(msg.bumper == 0)
        bumperLeft = !bumperLeft;
    else if(msg.bumper == 1)
        bumperCenter = !bumperCenter;
    else if(msg.bumper == 2)
        bumperRight = !bumperRight;

}

void cliffCB(const kobuki_msgs::CliffEvent msg){
	if(msg.state == 1){
		lift = 1;
	}
	else
		lift = 0;
	
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;
	std::chrono::time_point<std::chrono::system_clock> start_time;
	std::chrono::time_point<std::chrono::system_clock> reset_time;
	start_time = std::chrono::system_clock::now();

	uint64_t passed_time = 0;
	uint64_t reset_duration = 0;

	Mat image1 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/sad.jpg");
	Mat image2 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/what.jpg");
	Mat image3 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/cheer.png");
	Mat image5 = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/fine.jpg");

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	//ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;


	geometry_msgs::Twist vel;


	//sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	namedWindow( "fine", WINDOW_NORMAL );
	imshow("fine", image5);
	setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
	setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_NORMAL);
	waitKey(50);
	setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
	waitKey(50);
	//
	imshow("fine", image5);
	setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
	waitKey(50);
	//


	while(ros::ok()){
		ros::spinOnce();

		if (follow_cmd.linear.x == 0 && last_linear_cmd == 0 && passed_time > 3){
			got_lost_counter ++;
		}

		if (back_n_forth_counter == 1){
			reset_time = std::chrono::system_clock::now();
			timer_on = 1;
		}

		if (follow_cmd.linear.x >= 0.1){
			direction = 1;
		}

		if (follow_cmd.linear.x <= -0.1){
			direction = 2;
		}

		if ( direction != last_direction && last_direction != 0){
			back_n_forth_counter ++;
		}
		
		last_direction = direction;

		last_linear_cmd = follow_cmd.linear.x;

		if (got_lost_counter >= 20000){
			got_lost_counter = 0;
			got_lost = 1;
		}

		if (reset_duration > 9){
			back_n_forth_counter = 0;
			reset_duration = 0;
			timer_on = 0;
		}

		if (back_n_forth_counter >= 5){
			back_n_forth_counter = 0;
			back_n_forth = 1;
			last_direction = 0;
		}



		//check if turlebot is lifted:
		if (lift == 1){
			world_state = 1;
			//if 
		}
		//check if turlebot hit something:
		else if (bumperCenter || bumperLeft || bumperRight){
			world_state = 2;
		}
		//check if turlebot was forced to move back and forth:
		else if (back_n_forth){
			world_state = 3;
			back_n_forth = 0;
		}
		//check if turlebot lost sight of the person:
		else if (got_lost){
			world_state = 4;
			got_lost = 0;
		}
		else {
			world_state = 0;
		}



//DETERMINE WORLD STATE

		if(world_state == 0){
			//fill with your code
			//ROS_INFO_THROTTLE(0.1, "I'm good!!!");
			//good_counter ++;
			vel_pub.publish(follow_cmd);
			

		}
		else if (world_state == 1){


			namedWindow( "cheer", WINDOW_NORMAL );
			imshow("cheer", image3);
			setWindowProperty("cheer",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			setWindowProperty("cheer",WND_PROP_FULLSCREEN,WINDOW_NORMAL);
			waitKey(50);
			setWindowProperty("cheer",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
			//
			imshow("cheer", image3);
			setWindowProperty("cheer",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
			//

			sc.playWave(path_to_sounds + "kids_cheering.wav");
			sleep(4.0);
			sc.stopWave(path_to_sounds + "kids_cheering.wav");
		}
		else if (world_state == 2){
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);

			

			namedWindow( "what", WINDOW_NORMAL );
			imshow("what", image2);
			setWindowProperty("what",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			setWindowProperty("what",WND_PROP_FULLSCREEN,WINDOW_NORMAL);
			waitKey(50);
			setWindowProperty("what",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
			//
			imshow("what", image2);
			setWindowProperty("what",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
			//

			
			sc.playWave(path_to_sounds + "what_happened.wav");
			sleep(2.0);
			sc.stopWave(path_to_sounds + "what_happened.wav");
		}
		else if (world_state == 3){
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);

			sc.playWave(path_to_sounds + "complaining.wav");
			sleep(6.0);
			sc.stopWave(path_to_sounds + "complaining.wav");
		}
		else if (world_state == 4){
			vel.angular.z = 0.5;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			

			sleep(1.0);

			vel.angular.z = -0.7;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);


			sleep(1.0);

			vel.angular.z = 0.5;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			


			sleep(1.0);

			namedWindow( "sad", WINDOW_NORMAL );
			imshow("sad", image1);
			setWindowProperty("sad",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			setWindowProperty("sad",WND_PROP_FULLSCREEN,WINDOW_NORMAL);
			waitKey(50);
			setWindowProperty("sad",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
			imshow("sad", image1);
			setWindowProperty("sad",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
	

			sc.playWave(path_to_sounds + "Crying.wav");
			sleep(3.0);
			sc.stopWave(path_to_sounds + "Crying.wav");
		}
		
		if (world_state != 0){

			namedWindow( "fine", WINDOW_NORMAL );
			imshow("fine", image5);
			setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_NORMAL);
			waitKey(50);
			setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
			//
			imshow("fine", image5);
			setWindowProperty("fine",WND_PROP_FULLSCREEN,WINDOW_FULLSCREEN);
			waitKey(50);
			//
		}
		

		
		
		
		ROS_INFO_THROTTLE(0.1, "direction flip counter: %d", back_n_forth_counter);
		//ROS_INFO_THROTTLE(0.1, "good counter: %d", good_counter);
		// ROS_INFO_THROTTLE(0.1, "GOT LOST COUNTER: %d", got_lost_counter);
		passed_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_time).count();
		
		if (timer_on){
			reset_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-reset_time).count();
			ROS_INFO_THROTTLE(0.1, "time duration: %d", reset_duration);
		}
		

	}

	return 0;
}
