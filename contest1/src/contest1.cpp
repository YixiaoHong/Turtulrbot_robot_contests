#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>
#include <stdio.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include<ctime>
#include <iostream>
#include <chrono>

using namespace std;

//odem variables
double posX, posY, yaw;
double pi=3.141593;

//laser variables
double laserRange = 10.0;
int laserSize = 0, laserOffset = 0, desiredAngle = 29;
double laserMid = 10.0;
double laserMidL0 = 10.0;
double laserMidR0 = 10.0;

double laserMidL = 10.0;
double laserMidR = 10.0;
double laserMidL2 = 10.0;
double laserMidR2 = 10.0;
double laserMidL3 = 10.0;
double laserMidR3 = 10.0;
double laserMidSum = 20.0;

double laserMostL = 10.0;
double laserMostR = 10.0;

int needLeftTurn = 1; //1 means need left turn, 0 means no need

//Bumper Variable
bool bumperLeft = 0, bumperCenter = 0, bumperRight =0;


//speed control
double angular = 0.0;
double linear = 0.0;
double tol = 10.0;


int lastPoint = 0;
int oneMeterFlag = 0;  //0 means still in 1m from last checkpoint, 1 means farther than 1m from last check point
double lastCheckX = 0.0;
double lastCheckY = 0.0;

//Turning Virables
double refAngle = 0.0;

//Half distance move virable
double longestDis = 0.0;
int leftRightScanFlag = 0;
double multiplier = 0.0;

double stopPointStack[100][4];
int stackPointer = 0;

//Stuck Check
double lastX = 0.0;
double lastY = 0.0;


//Bumper Call Back Function
void bumperCallback(const kobuki_msgs::BumperEvent msg){

    if(msg.bumper == 0)
        bumperLeft = !bumperLeft;
    else if(msg.bumper == 1)
        bumperCenter = !bumperCenter;
    else if(msg.bumper == 2)
        bumperRight = !bumperRight;

}


//Laser Call Back Function
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    laserOffset = desiredAngle*pi/(180*msg->angle_increment);
    laserRange = 11.0;

    if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 >msg->angle_min){
        for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
            if(laserRange > msg->ranges[i])
            laserRange = msg->ranges[i];
        }
    }
    else{
        for(int i=0; i < laserSize; i++){
            if(laserRange>msg->ranges[i])
                laserRange = msg->ranges[i];
        }
    }

    if(laserRange == 11.0){
        laserRange = 0.0;
    }


    //Scan Laser Mid
    for (int i=0; i<10 ; i++){
        laserMidL0 = msg->ranges[319+i];
        laserMidR0 = msg->ranges[319-i];
        if (isnan(laserMidL0) == 0 && isnan(laserMidR0) == 0 ){
            break;
        }
        else{
            laserMidL0 = 0.0;
            laserMidR0 = 0.0;
        }
    }
    laserMid = (laserMidL0+laserMidR0)/2;

    //Scan Left and Right with correction
    for (int i=0; i<10 ; i++){
        laserMidL = msg->ranges[319+80+i];
        laserMidR = msg->ranges[319-80-i];
        if (isnan(laserMidL) == 0 && isnan(laserMidR) == 0 ){
            break;
        }
        else{
            laserMidL = 0.0;
            laserMidR = 0.0;
        }
    }

    for (int i=0; i<10 ; i++){
        laserMidL2 = msg->ranges[319+150+i];
        laserMidR2 = msg->ranges[319-150-i];
        if (isnan(laserMidL2) == 0 && isnan(laserMidR2) == 0 ){
            break;
        }
        else{
            laserMidL2 = 0.0;
            laserMidR2 = 0.0;
        }
    }

    for (int i=0; i<10 ; i++){
        laserMidL3 = msg->ranges[319+280+i];
        laserMidR3 = msg->ranges[319-280-i];
        if (isnan(laserMidL3) == 0 && isnan(laserMidR3) == 0 ){
            break;
        }
        else{
            laserMidL3 = 0.0;
            laserMidR3 = 0.0;
        }
    }

    for (int i=0; i<15 ; i++){
        laserMostL = msg->ranges[629 -i];
        laserMostR = msg->ranges[9 + i];
        if (isnan(laserMostL) == 0 && isnan(laserMostR) == 0 ){
            break;
        }
        else{
            laserMostL = 0.0;
            laserMostR = 0.0;
        }
    }


    if (laserMostL > 0.6 && laserMostR >0.6 && laserRange >0.5){
        laserMidSum = laserMidL + laserMidR + 0.3*laserMidL2 + 0.3*laserMidR2 + 0.5*laserMidL3 + 0.5*laserMidR3 + laserMostL + laserMostR;
    }
    else {
        laserMidSum=0;
    }

}

//Odom Call Back Function
void odomCallback (const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

///////////////////////////////////////////////////////
//////      Self Defined Functions          ///////////
///////////////////////////////////////////////////////

//This Function corrects an angle when it exceed the range
double targetCorr(double target){
    if (target > pi){
        target -= 2*3.1415926;
    }
    if (target < -pi){
        target += 2*3.1415926;
    }
    return target;
}

//This Function returns the difference between target angle and current angle
double diffCheck(double target, double yaw){
    if (target > pi){
        target -= 2*3.1415926;
    }
    if (target < -pi){
        target += 2*3.1415926;
    }
        double Diff = target - yaw;
    if (Diff <= -pi){
        Diff += 2*3.1415926;
    }
    if (Diff >= pi){
        Diff -= 2*3.1415926;
    }

    //if (abs(Diff) < 0.0873){
    //Diff = 0;
    //}

    return Diff;
}

//This Function returns a tolerance between target angle and current angle
double tolCheck(double target , double yaw){
    double tol = target - yaw;
    if (tol < -pi){
    tol += 2*3.1415926;
    }
    if (tol > pi){
    tol -= 2*3.1415926;
    }
    return tol;
}

///////////////////////////////////////////////////////
/////////////           Main          /////////////////
///////////////////////////////////////////////////////

int main(int argc, char **argv){

    double refAngle = 0.0; //Initial Direction

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    teleController eStop;
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    std::chrono::time_point<std::chrono::system_clock> myTimerStart;
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> stuckTimerStart;
    start = std::chrono::system_clock::now(); /* start timer */
    stuckTimerStart = std::chrono::system_clock::now(); /* Timer that resets every 30s to determine if the rover get stuck in the same location for too long */
    uint64_t stuckTimeElapsed = 0;
    uint64_t secondsElapsed = 0;

    while(ros::ok() && secondsElapsed <= 480){

        ros::spinOnce();
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //..............................

        ////////////////////////////////
        /////   Stuck Check   //////////
        ///////////////////////////////
        
        if (stuckTimeElapsed > 30 ){

            if (sqrt(pow((lastX - posX),2) + pow((lastY - posY),2) ) < 0.2){
                
                ///////////////////////
                /// Back for 1 second /
                ///////////////////////

                myTimerStart = std::chrono::system_clock::now();
                uint64_t myTimerElapsed = 0;
                while(myTimerElapsed <= 1){
                    angular = 0.0;
                    linear = -0.05;
                    vel.angular.z = angular;
                    vel.linear.x = linear;
                    vel_pub.publish(vel);
                    myTimerElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-myTimerStart).count();
                }

                ///////////////////////
                //turn L 90 ///////////
                ///////////////////////

                ros::spinOnce();

                /////Ref Angle Set up
                double Diff = diffCheck(yaw + 1.57,yaw);
                refAngle = targetCorr(refAngle + 1.57);
                //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
                //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
                int done = 0;
                ////End of Target Angle Set up

                    /////////Left Turn
                    while (done == 0){
                        ros::spinOnce();
                        tol = tolCheck (refAngle, yaw);
                        if (abs(tol) >= 0.035){
                            vel.angular.z = pi/6; //CONST LEFT TURN
                            vel.linear.x = 0.0;
                            vel_pub.publish(vel);
                        }
                        else{
                            done = 1;
                        }
                        double currentLaserMid = laserMidSum; 
                    }

                    angular = 0.0;
                    linear = 0.0;
                    vel.angular.z = angular;
                    vel.linear.x = linear;
                    vel_pub.publish(vel);
                /////End of Left Turn


            }

            ros::spinOnce();
            stuckTimerStart = std::chrono::system_clock::now(); /* stuck timer */
            lastX = posX;
            lastY = posY;
        }

        ////////////////////////////////
       /////Stright Function///////////
       ///////////////////////////////
       if(!bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.5 && leftRightScanFlag == 0){

          
            
           angular = 0.0;
           linear = 0.2;
            if (secondsElapsed > 0){
                multiplier = 0;
            }
            if (secondsElapsed > 160){
                multiplier = 0.5;
            }
            if (secondsElapsed > 320){
                multiplier = 0.6666;
            }
            
           ros::spinOnce();
           if ((longestDis > 3) && (laserMid<(longestDis*multiplier))){
              
                vel.angular.z = 0.0; //STOP
                vel.linear.x = 0.0;
                vel_pub.publish(vel);
                leftRightScanFlag = 1;
                //cout << "leftRightScanFlag = 1\n";
              
            }

           double diff = diffCheck (refAngle,yaw);
      
            if (abs(diff)<0.15){
                angular = 0.0;
                linear = 0.2;
            }
            else {
                linear = 0.15;
                if (diff>0){
                    angular = pi/10;
                }
                else{
                    angular = -pi/10;
                }
            }

            if (laserMostL < 0.55){
                angular = -pi/12;
                linear = 0.2;
                refAngle = yaw;
            }
            if (laserMostR < 0.55){
                angular = pi/12;
                linear = 0.2;
                refAngle = yaw;
            }

            if (laserMid > longestDis){
                longestDis = laserMid;
                //cout << "longestDis now is "<<longestDis<<"\n";
            }

           /*
           if (((laserMostL + 0.5* laserMidL3) - (laserMostR + 0.5* laserMidR3) > 1.5) && (laserMostL > 0.65)){
               angular = pi/8;
               linear = 0.15;
               refAngle = yaw;


           }
           else if (((laserMostL + 0.5* laserMidL3) - (laserMostR + 0.5* laserMidR3) < -1.5) && (laserMostR > 0.65)){
               angular = -pi/8;
               linear = 0.15;
               refAngle = yaw;
           }
           */

       }

       ////////////////////////////////
       /////Left Right Scan Mode//////
       ///////////////////////////////
       else if (!bumperRight && !bumperCenter && !bumperLeft && leftRightScanFlag ==1){
          
           double longestLaserMid = 0.0;
           double longestLaserMidYaw = 0.0;
           longestDis = 0.0;
           double Diff;

           //////////////////////
           //////First Turn///////
           ///////////////////////

           ros::spinOnce();

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 0.7854,yaw);
           refAngle = targetCorr(refAngle + 0.7854);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           int done = 0;
           ////End of Target Angle Set up

                   /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);

           //cout<<"Perform First Turn"<<".\n";

          
           //////////////////////
           //////2nd  Turn///////
           ///////////////////////

           Diff = diffCheck(yaw + 1.57,yaw);
           refAngle = targetCorr(refAngle + 1.57);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           done = 0;
           ////End of Target Angle Set up

           /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }

                   ros::spinOnce();
                   double currentLaserMid = laserMidSum;
                   //cout<<"currentLaserMid= "<<currentLaserMid<<".\n";
                   if (currentLaserMid > longestLaserMid){
                       if (laserRange > 0.55){
                           longestLaserMid = currentLaserMid;
                           longestLaserMidYaw = yaw;
                           needLeftTurn = 0;
                           longestDis = laserMid;

                           //cout<<"1.LongestLaserMId= "<<longestLaserMid<<".\n";
                           //cout<<"1.LongestLaserMIdYaw= "<<longestLaserMidYaw<<".\n";
                       }
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);
           /////End of Left Turn



           //////////////////////
           //////3rd  Turn///////
           ///////////////////////

           //cout<<"Perform Second Turn"<<".\n";

           ros::spinOnce();

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 1.57,yaw);
           refAngle = targetCorr(refAngle + 1.57);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           done = 0;
           ////End of Target Angle Set up

                   /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);
           /////End of Left Turn


           //////////////////////
           //////4th  Turn///////
           ///////////////////////

           //cout<<"Perform Third Turn"<<".\n";

           ros::spinOnce();

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 1.57,yaw);
           refAngle = targetCorr(refAngle + 1.57);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           done = 0;
           ////End of Target Angle Set up

           /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }

                   ros::spinOnce();
                   double currentLaserMid = laserMidSum;
                   //cout<<"currentLaserMid= "<<currentLaserMid<<".\n";
                   if (currentLaserMid > longestLaserMid){
                      
                       //cout<<"Enter Determ Scan Loop"<<".\n";
                       //cout<<"currentLaserRange= "<<laserRange<<".\n";

                       if (laserRange > 0.55){
                           longestLaserMid = currentLaserMid;
                           longestLaserMidYaw = yaw;
                           needLeftTurn = 0;
                           longestDis = laserMid;

                           //cout<<"2.LongestLaserMId= "<<longestLaserMid<<".\n";
                           //cout<<"2.LongestLaserMIdYaw= "<<longestLaserMidYaw<<".\n";

                       }
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);
               //ROS_INFO("longestlaser mid yaw is: %f", (longestLaserMidYaw/3.14*180));
           /////End of Left Turn
          
           //////////////////////
           //////5th  Turn///////
           ///////////////////////

           ros::spinOnce();

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 0.7854,yaw);
           refAngle = targetCorr(refAngle + 0.7854);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           done = 0;
           ////End of Target Angle Set up

                   /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);

           //cout<<"Perform First Turn"<<".\n";

           leftRightScanFlag =0;
          
           if (longestDis > 1.3){
               ros::spinOnce();

               /////Ref Angle Set up
               refAngle = longestLaserMidYaw;
               Diff = diffCheck(refAngle, yaw);
               //ROS_INFO("currently facing %f degrees", (yaw/3.14*180));

               //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
               //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
               done = 0;
               ////End of Target Angle Set up

               //Turn Left or Right
               if (Diff>0){

                   //cout<<"Left closer to exit"<<".\n";

                   //Turn Left////
                   while (done == 0){
                       ros::spinOnce();
                       tol = tolCheck (refAngle, yaw);
                       if (abs(tol) >= 0.035){
                           vel.angular.z = pi/6; //CONST LEFT TURN
                           vel.linear.x = 0.0;
                           vel_pub.publish(vel);
                       }
                       else{
                           done = 1;
                       }
                   }
                   vel.angular.z = 0.0; //STOP
                   vel.linear.x = 0.0;
                   vel_pub.publish(vel);
                   /////End of Left Turn

               }

               else if (Diff<0){

                       //cout<<"Right closer to exit"<<".\n";

                   //Turn Right////
                   while (done == 0){
                       ros::spinOnce();
                       tol = tolCheck (refAngle, yaw);
                       if (abs(tol) >= 0.035){
                           vel.angular.z = -pi/6; //CONST Right TURN
                           vel.linear.x = 0.0;
                           vel_pub.publish(vel);
                       }
                       else{
                           done = 1;
                       }
                   }
                   vel.angular.z = 0.0; //STOP
                   vel.linear.x = 0.0;
                   vel_pub.publish(vel);
                   /////End of Left Turn

               }

           }
       }

       ////////////////////////////////
       /////Turing Function!///////////
       ///////////////////////////////
       else if(!bumperRight && !bumperCenter && !bumperLeft && laserRange <= 0.5) {
      
           //cout<<"=============Barrier Detected==============="<<".\n";
           needLeftTurn = 1; //0 no need, 1 need left 90
           ros::spinOnce();

           //Virables
           double longestLaserMid = 0.0;
           double longestLaserMidYaw = 0.0;
           longestDis = 0.0;
           double Diff;

           //////////////////////
           //////First Turn///////
           ///////////////////////

           //cout<<"Perform First Turn"<<".\n";

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 1.57,yaw);
           refAngle = targetCorr(refAngle + 1.57);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           int done = 0;
           ////End of Target Angle Set up

           /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }

                   ros::spinOnce();
                   double currentLaserMid = laserMidSum;
                   //cout<<"currentLaserMid= "<<currentLaserMid<<".\n";
                   if (currentLaserMid > longestLaserMid){
                       if (laserRange > 0.55){
                           longestLaserMid = currentLaserMid;
                           longestLaserMidYaw = yaw;
                           needLeftTurn = 0;
                           longestDis = laserMid;

                           //cout<<"1.LongestLaserMId= "<<longestLaserMid<<".\n";
                           //cout<<"1.LongestLaserMIdYaw= "<<longestLaserMidYaw<<".\n";
                       }
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);
           /////End of Left Turn



           //////////////////////
           //////Second Turn///////
           ///////////////////////

           //cout<<"Perform Second Turn"<<".\n";

           ros::spinOnce();

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 3.14159,yaw);
           refAngle = targetCorr(refAngle + 3.14159);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           done = 0;
           ////End of Target Angle Set up

                   /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);
           /////End of Left Turn


           //////////////////////
           //////Third Turn///////
           ///////////////////////

           //cout<<"Perform Third Turn"<<".\n";

           ros::spinOnce();

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 1.57,yaw);
           refAngle = targetCorr(refAngle + 1.57);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           done = 0;
           ////End of Target Angle Set up

           /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }

                   ros::spinOnce();
                   double currentLaserMid = laserMidSum;
                   //cout<<"currentLaserMid= "<<currentLaserMid<<".\n";
                   if (currentLaserMid > longestLaserMid){
                      
                       //cout<<"Enter Determ Scan Loop"<<".\n";
                       //cout<<"currentLaserRange= "<<laserRange<<".\n";

                       if (laserRange > 0.55){
                           longestLaserMid = currentLaserMid;
                           longestLaserMidYaw = yaw;
                           needLeftTurn = 0;
                           longestDis = laserMid;

                           //cout<<"2.LongestLaserMId= "<<longestLaserMid<<".\n";
                           //cout<<"2.LongestLaserMIdYaw= "<<longestLaserMidYaw<<".\n";

                       }
                   }
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);
               //ROS_INFO("longestlaser mid yaw is: %f", (longestLaserMidYaw/3.14*180));
           /////End of Left Turn


           //////////////////////
           //////Facing Longest Range Direction///////
           ///////////////////////
           if (needLeftTurn == 0){

               ros::spinOnce();

               /////Ref Angle Set up
               refAngle = longestLaserMidYaw;
               Diff = diffCheck(refAngle, yaw);
               //ROS_INFO("currently facing %f degrees", (yaw/3.14*180));

               //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
               //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
               done = 0;
               ////End of Target Angle Set up

               //Turn Left or Right
               if (Diff>0){

                   //cout<<"Left closer to exit"<<".\n";

                   //Turn Left////
                   while (done == 0){
                       ros::spinOnce();
                       tol = tolCheck (refAngle, yaw);
                       if (abs(tol) >= 0.035){
                           vel.angular.z = pi/6; //CONST LEFT TURN
                           vel.linear.x = 0.0;
                           vel_pub.publish(vel);
                       }
                       else{
                           done = 1;
                       }
                   }
                   vel.angular.z = 0.0; //STOP
                   vel.linear.x = 0.0;
                   vel_pub.publish(vel);
                   /////End of Left Turn

               }

               else if (Diff<0){

                       //cout<<"Right closer to exit"<<".\n";

                   //Turn Right////
                   while (done == 0){
                       ros::spinOnce();
                       tol = tolCheck (refAngle, yaw);
                       if (abs(tol) >= 0.035){
                           vel.angular.z = -pi/6; //CONST Right TURN
                           vel.linear.x = 0.0;
                           vel_pub.publish(vel);
                       }
                       else{
                           done = 1;
                       }
                   }
                   vel.angular.z = 0.0; //STOP
                   vel.linear.x = 0.0;
                   vel_pub.publish(vel);
                   /////End of Left Turn

               }

           }


           else{

           //////////////////////
           //////Turn Left 90 if no Avaliable direction///////
           ///////////////////////

           //ROS_INFO("There is no avaliable direction to go");
           //cout<<"There is no avaliable direction to go in the front, turn left 90 degree"<<".\n";

           ros::spinOnce();

           /////Ref Angle Set up
           Diff = diffCheck(yaw + 1.57,yaw);
           refAngle = targetCorr(refAngle + 1.57);
           //ROS_INFO("Target angle is: %f degrees", (refAngle/3.14*180));
           //ROS_INFO("Left turn: Diff is %f degress", (Diff/3.14*180));
           done = 0;
           ////End of Target Angle Set up

                   /////////Left Turn
               while (done == 0){
                   ros::spinOnce();
                   tol = tolCheck (refAngle, yaw);
                   if (abs(tol) >= 0.035){
                       vel.angular.z = pi/6; //CONST LEFT TURN
                       vel.linear.x = 0.0;
                       vel_pub.publish(vel);
                   }
                   else{
                       done = 1;
                   }
                   double currentLaserMid = laserMidSum;
               }
               vel.angular.z = 0.0; //STOP
               vel.linear.x = 0.0;
               vel_pub.publish(vel);
           /////End of Left Turn

           }//End of last else

        } //Wall

       ////////////////////////////////
       /////      Back      ///////////
       ///////////////////////////////
       else{

           myTimerStart = std::chrono::system_clock::now();
           uint64_t myTimerElapsed = 0;

           while(myTimerElapsed <= 1){
               angular = 0.0;
               linear = -0.05;
               vel.angular.z = angular;
               vel.linear.x = linear;
               vel_pub.publish(vel);
               myTimerElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-myTimerStart).count();
           }
       }

        //Vel Punblish
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        stuckTimeElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-stuckTimerStart).count();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    } //While Ros OK

    return 0;

}




