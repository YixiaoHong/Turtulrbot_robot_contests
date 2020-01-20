#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include <math.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

//initial position record, go back to this location at the end
double initialX;
double initialY;
double initialPhi;

//robot coordinate aray
double robotCoord[5][4];
double copyCoord[5][3];
double disToBox = 0.6; //in meter to box

//Manhattan Dis calculation
int bestPosIndex;
double currentDis;

//duplicate image checker
int dupCheck[4];

int closestPoint(double robotx, double roboty){
    
    double minimumDis = 99.9;

    for(int i = 0; i < 5; ++i) {
        if (robotCoord[i][3] == 0){

            currentDis = sqrt(pow((robotx-robotCoord[i][0]),2.0)+pow((roboty-robotCoord[i][1]),2.0));

            if (currentDis < minimumDis){
                bestPosIndex = i;
                minimumDis = currentDis;
            }

        }

    }

    return bestPosIndex ;

}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }

    //convert to robot cordinates
    for(int i = 0; i < boxes.coords.size(); ++i) {
        
        /*
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
        << boxes.coords[i][2] << std::endl;
        */
        
        robotCoord[i][0]=boxes.coords[i][0] + disToBox*cos(boxes.coords[i][2]); //X Pos
        robotCoord[i][1]=boxes.coords[i][1] + disToBox*sin(boxes.coords[i][2]); //Y Pos
        robotCoord[i][2]=boxes.coords[i][2] + 3.14159; //Yaw Pos
        if (robotCoord[i][2]>3.14159){
            robotCoord[i][2] = robotCoord[i][2] - 6.283;
        }

        std::cout << "Converted Robot coordinates: " << std::endl;
        std::cout << i << " x: " << robotCoord[i][0] << " y: " << robotCoord[i][1] << " z: " 
                  << robotCoord[i][2] << std::endl;

    }



    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    ros::Duration(2.0).sleep();//Delay looping prevent lag
    ros::spinOnce();
    initialX = robotPose.x;
    initialY = robotPose.y;
    initialPhi = robotPose.phi;
    cout <<"Initial X="<<initialX<<"Initial Y="<<initialY<<"Initial Yaw="<<initialPhi<<"\n";



    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        //record data intp txt file
        std::ofstream myfile;
        myfile.open ("record.txt");
        
        //go to type 1: possible to reach
        for(int i = 0; i < boxes.coords.size(); ++i) {

            int currentIndex = closestPoint(robotPose.x,robotPose.y);

            //Get to the index position
            cout <<"\n\n\nNow going to Pos"<<currentIndex<<"\n";
            cout <<"target X="<<robotCoord[currentIndex][0]<<"target Y="<<robotCoord[currentIndex][1]<<"target Yaw="<<robotCoord[currentIndex][2]<<"\n";
            if (Navigation:: moveToGoal(robotCoord[currentIndex][0],robotCoord[currentIndex][1],robotCoord[currentIndex][2]) == false){
                std::cout<<"\\n\nwrong location, giving it another chance\n\n";
                if (Navigation:: moveToGoal(robotCoord[currentIndex][0],robotCoord[currentIndex][1],robotCoord[currentIndex][2]) == false){
                    robotCoord[currentIndex][3] = 2;
                }
                else{
                    robotCoord[currentIndex][3] = 1;
                }
            }
            else{
                robotCoord[currentIndex][3] = 1;
            }

            ros::spinOnce();
            
            cout <<"Current X="<<robotPose.x<<"Current Y="<<robotPose.y<<"Current Yaw="<<robotPose.phi<<"\n";
            if (robotCoord[currentIndex][3] == 1){
                ros::spinOnce();
                int pictureID = imagePipeline.getTemplateID(boxes);
                if (pictureID == 0 || pictureID == 1 || pictureID == 2){
                    if (dupCheck[pictureID] == 1){
                        myfile << "This is duplicated, ";
                    }
                    dupCheck[pictureID] = 1;
                    if (pictureID == 0){
                        myfile << "The picture identified is: Raisin Bran.\n";
                        myfile <<"Box Location: X="<<boxes.coords[currentIndex][0]<<" Box Y="<<boxes.coords[currentIndex][1]<<" Box Direction="<<boxes.coords[currentIndex][2]<<"\n";
                        myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                    }
                    if (pictureID == 1){
                        myfile << "The picture identified is: Cinnamon Toast Crunch.\n";
                        myfile <<"Box Location: X="<<boxes.coords[currentIndex][0]<<" Box Y="<<boxes.coords[currentIndex][1]<<" Box Direction="<<boxes.coords[currentIndex][2]<<"\n";
                        myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                    }
                    if (pictureID == 2){
                        myfile << "The picture identified is: Rice Krispies.\n";
                        myfile <<"Box Location: X="<<boxes.coords[currentIndex][0]<<" Box Y="<<boxes.coords[currentIndex][1]<<" Box Direction="<<boxes.coords[currentIndex][2]<<"\n";
                        myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                    }
                }
                else if (pictureID == 3){
                myfile<<"The picture identified is: Blank.\n";
                myfile <<"Box Location: X="<<boxes.coords[currentIndex][0]<<" Box Y="<<boxes.coords[currentIndex][1]<<" Box Direction="<<boxes.coords[currentIndex][2]<<"\n";
                myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                }
            }
        }

        // check if identified as type 2: can't reach
        for(int i = 0; i < boxes.coords.size(); ++i){
            if(robotCoord[i][3] == 2){
                Navigation:: moveToGoal(robotCoord[i][0],robotCoord[i][1],robotCoord[i][2]);
                ros::spinOnce();
                int pictureID = imagePipeline.getTemplateID(boxes);
                if (pictureID == 0 || pictureID == 1 || pictureID == 2){
                    if (dupCheck[pictureID] == 1){
                        myfile << "This is duplicated, ";
                    }
                    dupCheck[pictureID] = 1;
                    if (pictureID == 0){
                        myfile << "The picture identified is: Raisin Bran.\n";
                        myfile <<"Box Location: X="<<boxes.coords[i][0]<<" Box Y="<<boxes.coords[i][1]<<" Box Direction="<<boxes.coords[i][2]<<"\n";
                        myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                    }
                    if (pictureID == 1){
                        myfile << "The picture identified is: Cinnamon Toast Crunch.\n";
                        myfile <<"Box Location: X="<<boxes.coords[i][0]<<" Box Y="<<boxes.coords[i][1]<<" Box Direction="<<boxes.coords[i][2]<<"\n";
                        myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                    }
                    if (pictureID == 2){
                        myfile << "The picture identified is: Rice Krispies.\n";
                        myfile <<"Box Location: X="<<boxes.coords[i][0]<<" Box Y="<<boxes.coords[i][1]<<" Box Direction="<<boxes.coords[i][2]<<"\n";
                        myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                    }
                }
                else if (pictureID == 3){
                myfile<<"The picture identified is: Blank.\n";
                myfile <<"Box Location: X="<<boxes.coords[i][0]<<" Box Y="<<boxes.coords[i][1]<<" Box Direction="<<boxes.coords[i][2]<<"\n";
                myfile <<"Robot Current X="<<robotPose.x<<" Current Y="<<robotPose.y<<" Current Yaw="<<robotPose.phi<<"\n\n";
                }
            }
        }
        myfile.close();
        /*
        for(int i = 0; i < boxes.coords.size(); ++i) {
            robotCoord[i][3] = 0;
        }
        */

        cout <<"Now bo back"<<"\n";
        if (Navigation:: moveToGoal(initialX,initialY,initialPhi) == false){
            Navigation:: moveToGoal(initialX,initialY,initialPhi);
        }
        ros::spinOnce();
        cout <<"Current X="<<robotPose.x<<"Current Y="<<robotPose.y<<"Current Yaw="<<robotPose.phi<<"\n";
        
        ros::Duration(1.0).sleep();//Delay looping prevent lag
        cout << "=================Done=================";
        return 0;
    }
    return 0;
}
