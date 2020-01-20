#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>
#include <exception>    

using namespace cv;
using namespace cv::xfeatures2d;


#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"
//#define IMAGE_TOPIC "camera/image"

double distance[4]; //Stores calculated distance b/w corners of bounding box
double angle_hor, angle_ver, angle_adj_1, angle_adj_2;	// Stores the angles between horizontal, verticle, and adjacent lines respectively
double area = 0;    //Stores calculated area of bounding box  

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    //cout << "sub is "<< sub << "\n";
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        for (int i = 0; i < 3; i++){
            ros::spinOnce();
            cv::Mat img_object = boxes.templates[i];
            cv::Mat img_scene = img;
            if( !img_object.data || !img_scene.data ){
                 std::cout<< " --(!) Error reading images " << std::endl; return -1; }

            //-- Steps 1 & 2: Detect the keypoints and calculate descriptors using SURF Detector
            int minHessian = 400;
            Ptr<SURF> detector = SURF::create(minHessian);
            std::vector<KeyPoint> keypoints_object, keypoints_scene;
            Mat descriptors_object, descriptors_scene;
            detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
            detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);
            //-- Step 3: Matching descriptor vectors using FLANN matcher
            FlannBasedMatcher matcher;
            std::vector< DMatch > matches;
            matcher.match( descriptors_object, descriptors_scene, matches );
            double max_dist = 0; double min_dist = 100;
            
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < descriptors_object.rows; i++ ){
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            //printf("-- Max dist : %f \n", max_dist );
            //printf("-- Min dist : %f \n", min_dist );

            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< DMatch > good_matches;
            for( int i = 0; i < descriptors_object.rows; i++ ){
                if( matches[i].distance < 3*min_dist ){
                    good_matches.push_back( matches[i]); 
                    }
            }
            Mat img_matches;
            drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
            good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
            std:: vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;
            for( int i = 0; i < good_matches.size(); i++ ){
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
            }

            //std::cout<< "size of obj is "<<sizeof(obj)<<"\n";
            //std::cout<< "size of scene is "<<sizeof(scene)<<"\n\n";
            
            if (sizeof(obj)<4 || sizeof(scene)<4){
                return 3;
            }

            Mat H = findHomography( obj, scene, RANSAC );
            if (H.empty()){
                return 3;
            }
            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0); 
            obj_corners[1] = cvPoint( img_object.cols, 0 );
            obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
            obj_corners[3] = cvPoint( 0, img_object.rows );
            std::vector<Point2f> scene_corners(4);
            perspectiveTransform( obj_corners, scene_corners, H);

            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0),scene_corners[1] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0),scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0),scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0),scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );


        
            //Calculate distance between scene corners
            distance[0] = sqrt(pow((scene_corners[1].x - scene_corners[0].x), 2) + pow((scene_corners[1].y - scene_corners[0].y), 2));
            distance[1] = sqrt(pow((scene_corners[3].x - scene_corners[2].x), 2) + pow((scene_corners[3].y - scene_corners[2].y), 2));
            distance[2] = sqrt(pow((scene_corners[2].x - scene_corners[1].x), 2) + pow((scene_corners[2].y - scene_corners[1].y), 2));
            distance[3] = sqrt(pow((scene_corners[3].x - scene_corners[0].x), 2) + pow((scene_corners[3].y - scene_corners[0].y), 2));		
            //Calculate the angle between opposite and adjacent lines of the bounding box using dot product: theta = acos(a.b/|a||b|)
            angle_ver = acos((abs(scene_corners[2].x-scene_corners[1].x)*abs(scene_corners[3].x-scene_corners[0].x)+abs(scene_corners[2].y-scene_corners[1].y)*abs(scene_corners[3].y-scene_corners[0].y))/(distance[2]*distance[3]))*(180.0/3.1416);
            angle_hor = acos((abs(scene_corners[1].x-scene_corners[0].x)*abs(scene_corners[2].x-scene_corners[3].x)+abs(scene_corners[1].y-scene_corners[0].y)*abs(scene_corners[2].y-scene_corners[3].y))/(distance[0]*distance[1]))*(180.0/3.1416);
            angle_adj_1 = acos(((scene_corners[2].x-scene_corners[1].x)*(scene_corners[1].x-scene_corners[0].x)+(scene_corners[2].y-scene_corners[1].y)*(scene_corners[1].y-scene_corners[0].y))/(distance[0]*distance[2]))*(180.0/3.1416);	
            angle_adj_2 = acos(((scene_corners[2].x-scene_corners[3].x)*(scene_corners[3].x-scene_corners[0].x)+(scene_corners[2].y-scene_corners[3].y)*(scene_corners[3].y-scene_corners[0].y))/(distance[1]*distance[3]))*(180.0/3.1416);	   
            //Calculate area of the bounding box
            area = distance[0]*distance[2];

            // if any number is undefined, reset to perfect
            if (std::isnan(angle_ver) == 1){
                angle_ver = 1;
            }
            if (std::isnan(angle_hor) == 1){
                angle_hor = 1;
            }
            if (std::isnan(angle_adj_1) == 1){
                angle_adj_1 = 90;
            }
            if (std::isnan(angle_adj_2) == 1){
                angle_adj_2 = 90;
            }
            imshow( "Good Matches & Object detection", img_matches );
            std::cout<<"area is "<<area<<"\n";
            std::cout<<"good_matches.size() is "<<good_matches.size()<<"\n";
            std::cout<<"angle_ver is "<<angle_ver<<"\n";
            std::cout<<"angle_hor is "<<angle_hor<<"\n";
            std::cout<<"angle_adj_1 is "<<angle_adj_1<<"\n";
            std::cout<<"angle_adj_2 is "<<angle_adj_2<<"\n";

            //Check if the angle between opposite lines is close to zero and if it is, check if area calc is meaningful	
	        if((area > 5000.0) && (area < 100000.0) && (good_matches.size() >20) && (good_matches.size()<200) && (angle_ver < 15.0) && (angle_ver >= 0) && (angle_hor < 15.0) && (angle_hor >= 0) && (angle_adj_1 > 75.0) && (angle_adj_1 < 105.0) && (angle_adj_2 > 75.0) && (angle_adj_2 < 105.0)){
                cv::imshow("view", img);
                cv::waitKey(10);
                return i;
            }
        }
        cv::imshow("view", img);
        cv::waitKey(10);
        return 3;
    }  
    return template_id;
}



