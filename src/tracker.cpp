#include <iostream>
#include <vector>
#include <algorithm> 

#include <neural_cam_ros/obstacle.h>
#include <neural_cam_ros/obstacleStack.h>
//#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

#include <dlib/optimization/max_cost_assignment.h>

#define MAX_THRESH 5
#define IOU_THRESH 0.6 

using namespace std;
using namespace cv;
using namespace dlib;

// <--------- kalman filter parameters --------->
int stateSize = 6;
int measSize = 4;
int contrSize = 0;
unsigned int type = CV_32F;
double ticks = 0;

// callback controller
bool first = true;
int noObjectsCount = 0;

// datatype
typedef struct {
    int id;
    bool found = false;
    neural_cam_ros::obstacle object_data;

    KalmanFilter kf;
    Mat objectState;
    Mat objectMeas;
} object;


// container
std::vector <object> prev_objects;
std::vector <object> curr_objects;


float calculate_iou(cv::Point tl_a, cv::Point tl_b, cv::Point br_a, cv::Point br_b){

	//BBox A:
	float bbox_a_tl_x = (float) tl_a.x;
	float bbox_a_tl_y = (float) tl_a.y;
	float bbox_a_br_x = (float)	br_a.x;
	float bbox_a_br_y = (float)	br_a.y;

	//BBox B:
	float bbox_b_tl_x = (float) tl_b.x;
	float bbox_b_tl_y = (float) tl_b.y;
	float bbox_b_br_x = (float) br_b.x;
	float bbox_b_br_y = (float) br_b.y;

	float xA = max(bbox_a_tl_x,bbox_b_tl_x);
	float yA = max(bbox_a_tl_y,bbox_b_tl_y);
	float xB = max(bbox_a_br_x,bbox_b_br_x);
	float yB = max(bbox_a_br_y,bbox_b_br_y);

	float inter_area = (xB - xA)*(yB - yA);

	float boxAArea = (bbox_a_br_x - bbox_a_tl_x)*(bbox_a_br_y - bbox_a_tl_y);
	float boxBArea = (bbox_b_br_x - bbox_b_tl_x)*(bbox_b_br_y - bbox_b_tl_y);

	cout << "Intersectional Area: " << inter_area << endl;
	cout << "Box A area: " << boxAArea << endl;
	cout << "Box B area: " << boxBArea << endl;

	float iou = inter_area / (boxBArea + boxAArea - inter_area);

	cout << "Union Area: " << (boxBArea + boxAArea - inter_area) << endl;

	cout << "IOU: " << iou << endl;

	return iou;
}



void subCallback(const neural_cam_ros::obstacleStack::ConstPtr& msg)
{
   //ROS_INFO("Hearing From: [%s]", msg->stack_name.c_str());

   double precTick = ticks;
   ticks = (double) cv::getTickCount();

   double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

   int objects_detected = msg->stack_len;

   if(objects_detected > 0 && first){

   		//start initial assignment
   		for(int i = 0; i < objects_detected; i++){

   			object temp;

   			temp.id = i+1;				// assign an id to each object
   			temp.object_data = msg->stack_obstacles[i];
   			temp.found = true;

   			temp.kf.init(stateSize, measSize, contrSize, type);

	   		Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	  		Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

	  		temp.objectState = state;
	  		temp.objectMeas = meas;

	  		//Mat procNoise(stateSize, 1, type)
	  		// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

			// Transition State Matrix A
			// Note: set dT at each processing step!
			// [ 1 0 dT 0  0 0 ]
			// [ 0 1 0  dT 0 0 ]
			// [ 0 0 1  0  0 0 ]
			// [ 0 0 0  1  0 0 ]
			// [ 0 0 0  0  1 0 ]
			// [ 0 0 0  0  0 1 ]

			setIdentity(temp.kf.transitionMatrix);

			// Measure Matrix H
			// [ 1 0 0 0 0 0 ]
			// [ 0 1 0 0 0 0 ]
			// [ 0 0 0 0 1 0 ]
			// [ 0 0 0 0 0 1 ]

			temp.kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
			temp.kf.measurementMatrix.at<float>(0) = 1.0f;
			temp.kf.measurementMatrix.at<float>(7) = 1.0f;
			temp.kf.measurementMatrix.at<float>(16) = 1.0f;
			temp.kf.measurementMatrix.at<float>(23) = 1.0f;

			// Process Noise Covariance Matrix Q
			// [ Ex 0  0    0 0    0 ]
			// [ 0  Ey 0    0 0    0 ]
			// [ 0  0  Ev_x 0 0    0 ]
			// [ 0  0  0    1 Ev_y 0 ]
			// [ 0  0  0    0 1    Ew ]
			// [ 0  0  0    0 0    Eh ]

			//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
			temp.kf.processNoiseCov.at<float>(0) = 1e-2;
			temp.kf.processNoiseCov.at<float>(7) = 1e-2;
			temp.kf.processNoiseCov.at<float>(14) = 2.0f;
			temp.kf.processNoiseCov.at<float>(21) = 1.0f;
			temp.kf.processNoiseCov.at<float>(28) = 1e-2;
			temp.kf.processNoiseCov.at<float>(35) = 1e-2;

			// Measures Noise Covariance Matrix R
			setIdentity(temp.kf.measurementNoiseCov, cv::Scalar(1e-1));

   			prev_objects.push_back(temp);
   		}

   		first = false;

   }else if (objects_detected == 0 && !first){

   		// use previous tracking (previous IDs / kalman filter etc)
   		noObjectsCount++;

   		if(noObjectsCount > MAX_THRESH)	//track previous assignment for 5 frames if no object detected in scene
   		   first = true; 				//reset
   		else{

   			unsigned prev_num = prev_objects.size();

   			for(int i = 0; i < prev_num; i++){
   				prev_objects[i].kf.statePost = prev_objects[i].objectState;
   			}
		   
   		}

   }else if (objects_detected > 0 && !first){

   		/* 
   		 update assignment based on current measurement (hungarian)

   		 if current detected window count less than the previous window count, do object compensation 
   		*/

   		// store objects into the curr_objects
   		object contain_data;
   		for(int i = 0; i < objects_detected; i++){
   			contain_data.object_data = msg->stack_obstacles[i];
   			curr_objects.push_back(contain_data);
   		}

   		// getting object size
   		int size_prev_objects = prev_objects.size();

   		// cost matrix: row represent prev measurement, col represent curr measurement
		// [ c_ 0  0  0  0 0 ]
		// [ 0  c_ 0  0  0 0 ]
		// [ 0  0  c_ 0  0 0 ]
		// [ 0  0  0  c_ 0 0 ]
		// [ 0  0  0  0  1 c_]
		// [ 0  0  0  0  0 c_]

   		matrix<float> cost(size_prev_objects,size_prev_objects);

   		if(objects_detected <= size_prev_objects){       // Case 1: When detected objects in the current window less or equal to the previous detected

   			cout << "Case 1" << endl;

	   		for(int i = 0; i < size_prev_objects; i++){

	   			// calculate the IOU for each assignment 
	   			// if current detected objects is less than the prev_objects, assign dummy objects and set IOU as zero
	   			// if current detected objects more than the prev_objects, add into the new pile

	   			cv::Point A_tl;
	   			cv::Point A_br;

	   			A_tl.x = (int) prev_objects[i].object_data.topleft.x;
				A_br.x = (int) prev_objects[i].object_data.bottomright.x;
				A_tl.y = (int) prev_objects[i].object_data.topleft.y;
				A_br.y = (int) prev_objects[i].object_data.bottomright.y;

				//cout << "Previous " << i << " TL: " << A_tl << endl;
				//cout << "Previous " << i << " BR: " << A_br << endl;

	   			for(int j = 0; j < size_prev_objects; j++){	//check which give the highest IOU

	   				if(j >= objects_detected){

	   					cost(i,j) = -1;				//assign dummy value

	   				}else{
	   					cv::Point B_tl;
	   					cv::Point B_br;

	   					// calculate intersection area
	   					B_tl.x = (int) msg->stack_obstacles[i].topleft.x;
	   					B_br.x = (int) msg->stack_obstacles[i].bottomright.x;
	   					B_tl.y = (int) msg->stack_obstacles[i].topleft.y;
	   					B_br.y = (int) msg->stack_obstacles[i].bottomright.y;

	   					//cout << "Current " << j << " TL: " << B_tl << endl;
						//cout << "Current " << j << " BR: " << B_br << endl;

	   					cost(i,j) = calculate_iou(A_tl, B_tl, A_br, B_br);

	   				}
	   			}
	   		}

	   	}else{					// Case 2: When detected objects in the current window is strictly more than the previous detected

	   		cout << "Case 2" << endl;

	   		for(int i = 0; i < objects_detected; i++){

	   			cv::Point A_tl;
	   			cv::Point A_br;

	   			A_tl.x = (int) prev_objects[i].object_data.topleft.x;
				A_br.x = (int) prev_objects[i].object_data.bottomright.x;
				A_tl.y = (int) prev_objects[i].object_data.topleft.y;
				A_br.y = (int) prev_objects[i].object_data.bottomright.y;

	   			for(int j = 0; j < objects_detected; j++){	//check which give the highest IOU

	   				if(i >= size_prev_objects){

	   					cost(i,j) = -1;				//assign dummy value

	   				}else{
	   					cv::Point B_tl;
	   					cv::Point B_br;

	   					// calculate intersection area
	   					B_tl.x = (int) msg->stack_obstacles[i].topleft.x;
	   					B_br.x = (int) msg->stack_obstacles[i].bottomright.x;
	   					B_tl.y = (int) msg->stack_obstacles[i].topleft.y;
	   					B_br.y = (int) msg->stack_obstacles[i].bottomright.y;

	   					cost(i,j) = calculate_iou(A_tl, B_tl, A_br, B_br);

	   				}
	   			}
	   		}

	   	}


	   	prev_objects.swap(curr_objects);
	   	curr_objects.clear();

   }
}

/*
void testCallback(const neural_cam_ros::obstacleStack::ConstPtr& msg){

	ROS_INFO("Hearing From: [%s]", msg->stack_name.c_str());
	ROS_INFO("Hearing From: [%d]", msg->stack_len);

	cv::Point tl;
	cv::Point br;

	for(int i = 0; i < msg->stack_len; i++){
		tl.x = (int) msg->stack_obstacles[i].topleft.x;
		br.x = (int) msg->stack_obstacles[i].bottomright.x;
		tl.y = (int) msg->stack_obstacles[i].topleft.y;
		br.y = (int) msg->stack_obstacles[i].bottomright.y;
	}
	
}
*/


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera1/usb_cam1/obstacles", 1000, subCallback);

  ros::spin();

  return 0;
}