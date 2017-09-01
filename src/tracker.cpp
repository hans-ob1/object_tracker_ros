#include <iostream>
#include <vector>
#include <algorithm> 
#include <random>

#include <neural_cam_ros/obstacle.h>
#include <neural_cam_ros/obstacleStack.h>
//#include <thread>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

//#include <dlib/optimization/max_cost_assignment.h>

#include "Hungarian.h"

#define MAX_THRESH 10
#define IOU_MAX 99999
#define IOU_SCALING_FACTOR 1000 

using namespace std;
//using namespace dlib;
using namespace cv;

// <--------- kalman filter parameters --------->
int stateSize = 6;
int measSize = 4;
int contrSize = 0;
unsigned int type = CV_32F;
double ticks = 0;

//random number generator
std::random_device rd;
std::mt19937 mt(rd()); 							 	// seed the Mersenne Twister generator
std::uniform_int_distribution<> dist(100, 999); 	// define the range

// callback controller
bool setFlag = true;
int noObjectsCount = 0;

// Hungarian algo
HungarianAlgorithm HungAlgo;

// datatype struct
typedef struct {

    int id;					
    int notFoundCounter = 0;
    string label;

    cv::Point2f topLeft;
    cv::Point2f bottomRight;

    KalmanFilter kalmanTracker;
    Mat objectState;
    Mat objectMeas;

} object;

// container
std::vector <object> prev_objects;
std::vector <object> curr_objects;


// generate a random 4 digit number
int generateRandomID(string nametag){

	// first number is fixed:
	// 1 - pedestrian
	// 2 - bike
	// 3 - vehicle

	int front_tag, back_tag;

	if(nametag == "pedestrian")
		front_tag = 1000;
	else if (nametag == "bike")
		front_tag = 2000;
	else
		front_tag = 3000;

	back_tag = dist(mt);

	return (front_tag + back_tag);
}

KalmanFilter create_kalmanTracker(){

	KalmanFilter kf;

	// initialize kalman parameters	
	kf.init(stateSize, measSize, contrSize, type);

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

	setIdentity(kf.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]

	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex 0  0    0 0    0 ]
	// [ 0  Ey 0    0 0    0 ]
	// [ 0  0  Ev_x 0 0    0 ]
	// [ 0  0  0    1 Ev_y 0 ]
	// [ 0  0  0    0 1    Ew ]
	// [ 0  0  0    0 0    Eh ]

	cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf.processNoiseCov.at<float>(0) = 1e-2;
	kf.processNoiseCov.at<float>(7) = 1e-2;
	kf.processNoiseCov.at<float>(14) = 2.0f;
	kf.processNoiseCov.at<float>(21) = 1.0f;
	kf.processNoiseCov.at<float>(28) = 1e-2;
	kf.processNoiseCov.at<float>(35) = 1e-2;

	// Measures Noise Covariance Matrix R
	setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

	return kf;
}

// hieuristic function 
float calculate_iou(cv::Point2f tl_a, cv::Point2f tl_b, cv::Point2f br_a, cv::Point2f br_b){

	//check if the two box intersects
	//Only works if two rectangle intersects
	float iou;

		float xA = max(tl_a.x,tl_b.x);
		float yA = max(tl_a.y,tl_b.y);
		float xB = min(br_a.x,br_b.x);
		float yB = min(br_a.y,br_b.y);

		float inter_area = (xB - xA + 1)*(yB - yA + 1);

		float boxAArea = (br_a.x - tl_a.x + 1)*(br_a.y - tl_a.y + 1);
		float boxBArea = (br_b.x - tl_b.x + 1)*(br_b.y - tl_b.y + 1);

		iou = inter_area / (boxBArea + boxAArea - inter_area);

	return iou;
}


/***********************************************
		Spontaneous Callback Function
************************************************/

void subCallback(const neural_cam_ros::obstacleStack::ConstPtr& msg){

   // time keeping per cycle
   double precTick = ticks;
   ticks = (double) cv::getTickCount();
   double dT = (ticks - precTick) / cv::getTickFrequency();  //seconds

   // keep track of the number of objects in current/previous frame
   int num_objects_det_curr = msg->stack_len;
   int num_objects_det_prev = (int) prev_objects.size();

   // initialize the basic elements of detected objects
   for(int i = 0; i < num_objects_det_curr; i++){

   		object tempContainer;

   		// initialize label information
   		tempContainer.label = msg->stack_obstacles[i].name;

   		// initialize bbox coordinates
	   	tempContainer.topLeft.x = (float) msg->stack_obstacles[i].topleft.x;
	   	tempContainer.topLeft.y = (float) msg->stack_obstacles[i].topleft.y;
	   	tempContainer.bottomRight.x = (float) msg->stack_obstacles[i].bottomright.x;
	   	tempContainer.bottomRight.y = (float) msg->stack_obstacles[i].bottomright.y;

	   	// initialize dummy matrix
	   	cv::Mat state_set(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
		cv::Mat meas_set(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

		// set state/measurement size
	   	tempContainer.objectState = state_set;
	   	tempContainer.objectMeas = meas_set;

		// initialize measurement
		tempContainer.objectMeas.at<float>(0) = tempContainer.topLeft.x + (tempContainer.bottomRight.x - tempContainer.topLeft.x)/2;
		tempContainer.objectMeas.at<float>(1) = tempContainer.topLeft.y + (tempContainer.bottomRight.y - tempContainer.topLeft.y)/2;
		tempContainer.objectMeas.at<float>(2) = tempContainer.bottomRight.x - tempContainer.topLeft.x;;
		tempContainer.objectMeas.at<float>(3) = tempContainer.bottomRight.y - tempContainer.topLeft.y;

		// store into current objects
		curr_objects.push_back(tempContainer);
   }


   if(setFlag){   // flag toggle for first time seen objects

   		if(num_objects_det_curr > 0){   // objects detected for the first time!

   			for(int i = 0; i < num_objects_det_curr; i++){

   				// initialize notFoundcounter
   				curr_objects[i].notFoundCounter = 0;

   				// initialize name ID
   				curr_objects[i].id = generateRandomID(curr_objects[i].label);

				// initialize starting state
				curr_objects[i].objectState.at<float>(0) = curr_objects[i].topLeft.x + (curr_objects[i].bottomRight.x - curr_objects[i].topLeft.x)/2;
				curr_objects[i].objectState.at<float>(1) = curr_objects[i].topLeft.y + (curr_objects[i].bottomRight.y - curr_objects[i].topLeft.y)/2;
				curr_objects[i].objectState.at<float>(2) = 0;
				curr_objects[i].objectState.at<float>(3) = 0;				
				curr_objects[i].objectState.at<float>(4) = curr_objects[i].bottomRight.x - curr_objects[i].topLeft.x;;
				curr_objects[i].objectState.at<float>(5) = curr_objects[i].bottomRight.y - curr_objects[i].topLeft.y;

				// create kalman tracker
				curr_objects[i].kalmanTracker = create_kalmanTracker();
   			}

   			setFlag = false;
   		}

   }else{		// for subsequent frames


   		bool situation = true;
   		int max_mat_dim = max(num_objects_det_curr, num_objects_det_prev);

   		//matrix<int> cost(max_mat_dim,max_mat_dim);
   		vector< vector<double> > costMatrix;

   		if(num_objects_det_curr <= num_objects_det_prev){       	// Case 1: When detected objects in the current window less or equal to the previous detected

   			//cout << "Case 1: curr <= prev" << endl;

	   		for(int i = 0; i < num_objects_det_prev; i++){

	   			vector<double> rowStorage;

	   			for(int j = 0; j < num_objects_det_prev; j++){		//check which give the highest IOU

	   				if(j >= num_objects_det_curr){

	   					// missing detections in the current frame
	   					//cost(i,j) = -1*IOU_SCALING_FACTOR;								//assign dummy value

	   					rowStorage.push_back(IOU_MAX);

	   				}else{

	   					// calculate the IOU for each assignment 
	   					//loat iou_value = calculate_iou(prev_objects[i].topLeft, curr_objects[j].topLeft, prev_objects[i].bottomRight, curr_objects[j].bottomRight);

	   					//cost(i,j) = (int) (iou_value*IOU_SCALING_FACTOR);

	   					double iou_value = (double) IOU_SCALING_FACTOR/calculate_iou(prev_objects[i].topLeft, curr_objects[j].topLeft, prev_objects[i].bottomRight, curr_objects[j].bottomRight);

	   					rowStorage.push_back(iou_value);

	   				}
	   			}

	   			costMatrix.push_back(rowStorage);

	   		}

	   	}else{													// Case 2: When detected objects in the current window is strictly more than the previous detected

	   		situation = false;

	   		//cout << "Case 2: prev <= curr" << endl;

	   		for(int i = 0; i < num_objects_det_curr; i++){ 			//looking at previous objects

	   			vector<double> rowStorage;

		   		for(int j = 0; j < num_objects_det_curr; j++){   			

	   				if(i >= num_objects_det_prev){

	   					// cost(i,j) = -1*IOU_SCALING_FACTOR;				//assign dummy value

	   					rowStorage.push_back(IOU_MAX);

	   				}else{

		   				// float iou_value = calculate_iou(prev_objects[i].topLeft, curr_objects[j].topLeft, prev_objects[i].bottomRight, curr_objects[j].bottomRight);

		   				// cost(i,j) = (int) (iou_value*IOU_SCALING_FACTOR);

		   				double iou_value = (double) IOU_SCALING_FACTOR/calculate_iou(prev_objects[i].topLeft, curr_objects[j].topLeft, prev_objects[i].bottomRight, curr_objects[j].bottomRight);

		   				rowStorage.push_back(iou_value);

		   			}
	   			}

	   			costMatrix.push_back(rowStorage);
	   		}
	   	}


	   	// Reference: max_cost_assignment_ex.cpp
	   	// std::vector<long> assignment = dlib::max_cost_assignment(cost);			//do Hungarian assignment

	   	
		vector<int> assignment;
		double cost = HungAlgo.Solve(costMatrix, assignment);

    	for (unsigned int i = 0; i < assignment.size(); i++){

        	// cout << "Assignment: " << (int) assignment[i] << std::endl;

        	// int assignment_to = (int) assignment[i];

        	cout << "Assignment: " << assignment[i] << std::endl;

        	int assignment_to = assignment[i];

        	if(situation){

        		// if current detected objects less than the previous detected ones, assign the current detected ones with previous id
        		// mark the rest as undetected for this current iteration

        		if(assignment_to >= num_objects_det_curr) {		// assigned to hoax object

        			// mark as UNDETECTED OBJECTS for this round

        			if(prev_objects[i].notFoundCounter < MAX_THRESH){ 	//assess if the previous item need to be included into the current object list or not

        				// kalman estimate here:
        				prev_objects[i].kalmanTracker.statePost = prev_objects[i].objectState;

        				prev_objects[i].notFoundCounter++;
        				curr_objects.push_back(prev_objects[i]);
        			}

        		}else{		

					curr_objects[assignment_to].kalmanTracker = prev_objects[i].kalmanTracker;
					curr_objects[assignment_to].kalmanTracker.correct(curr_objects[assignment_to].objectMeas);		//correction

					if(curr_objects[assignment_to].label == prev_objects[i].label)
        				curr_objects[assignment_to].id = prev_objects[i].id;
        			else
        				curr_objects[assignment_to].id = generateRandomID(curr_objects[assignment_to].label);
        		}

        	}else{

        	 	if(i >= num_objects_det_prev){

        	 		// mark as NEW OBJECTS detected (generate new id/kalman filter)
        	 		curr_objects[assignment_to].id = generateRandomID(curr_objects[assignment_to].label);
        	 		curr_objects[assignment_to].notFoundCounter = 0;

					// state intialization
        			curr_objects[assignment_to].objectState.at<float>(0) = curr_objects[assignment_to].topLeft.x + (curr_objects[assignment_to].bottomRight.x - curr_objects[assignment_to].topLeft.x)/2;
					curr_objects[assignment_to].objectState.at<float>(1) = curr_objects[assignment_to].topLeft.y + (curr_objects[assignment_to].bottomRight.y - curr_objects[assignment_to].topLeft.y)/2;
					curr_objects[assignment_to].objectState.at<float>(2) = 0;
					curr_objects[assignment_to].objectState.at<float>(3) = 0;
					curr_objects[assignment_to].objectState.at<float>(4) = curr_objects[assignment_to].bottomRight.x - curr_objects[assignment_to].topLeft.x;
					curr_objects[assignment_to].objectState.at<float>(5) = curr_objects[assignment_to].bottomRight.y - curr_objects[assignment_to].topLeft.y;

					// insert new kalman filter here
					curr_objects[assignment_to].kalmanTracker = create_kalmanTracker();

        	 	}else{	//found and update assignment

					curr_objects[assignment_to].kalmanTracker = prev_objects[i].kalmanTracker;
					curr_objects[assignment_to].kalmanTracker.correct(curr_objects[assignment_to].objectMeas);		//correction

					if(curr_objects[assignment_to].label == prev_objects[i].label)
        				curr_objects[assignment_to].id = prev_objects[i].id;
        			else
        				curr_objects[assignment_to].id = generateRandomID(curr_objects[assignment_to].label);
        	 	}

        	}
    	}


    	/******************************
    			Display On Screen
    	******************************/
    	cv::Mat display_img(cv::Size(640, 480), CV_8UC3, Scalar(0,0,0));	//dummy display

    	for(int k = 0; k < curr_objects.size(); k++){
    		// >>>> Matrix A
      		curr_objects[k].kalmanTracker.transitionMatrix.at<float>(2) = dT;
      		curr_objects[k].kalmanTracker.transitionMatrix.at<float>(9) = dT;
      		// <<<< Matrix A

      		curr_objects[k].objectState = curr_objects[k].kalmanTracker.predict();

      		cv::Rect predRect;
      		predRect.width = curr_objects[k].objectState.at<float>(4);
      		predRect.height = curr_objects[k].objectState.at<float>(5);
      		predRect.x = curr_objects[k].objectState.at<float>(0) - predRect.width / 2;
      		predRect.y = curr_objects[k].objectState.at<float>(1) - predRect.height / 2;

      		// label the box
      		stringstream ss;
			ss << curr_objects[k].id;

			string curr_object_id = ss.str();
			Scalar object_colour;

			if(curr_objects[k].label == "pedestrian"){

				curr_object_id = "p: " + curr_object_id;
				object_colour = Scalar(255,0,0);
			}
			else if (curr_objects[k].label == "bike"){

				curr_object_id = "b: " + curr_object_id;
				object_colour = Scalar(0,0,255);
			}
			else{

				curr_object_id = "v: " + curr_object_id;
				object_colour = Scalar(0,255,0);
			}

       		cv::Point2f namePos(predRect.tl().x,predRect.tl().y-10);
       		cv::putText(display_img, curr_object_id, namePos, FONT_HERSHEY_PLAIN, 2.0, object_colour, 1.5);

      		cv::rectangle(display_img, predRect, object_colour, 2);
    	}

    	cv::imshow("testing", display_img);
    	cv::waitKey(5);

   }

    // store the current objects and clear
	prev_objects.swap(curr_objects);
	curr_objects.clear();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera1/usb_cam1/obstacles", 1000, subCallback);

  ros::spin();

  return 0;
}
