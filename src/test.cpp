#include <iostream>
#include <vector>
#include <algorithm> 

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

#include <dlib/optimization/max_cost_assignment.h>

#define MAX_THRESH 5
#define IOU_THRESH 0.6 

using namespace std;
using namespace dlib;

// callback controller
bool setflag = true;
int noObjectsCount = 0;

// datatype
typedef struct {
    int id;
    int notFoundCounter = 0;
    cv::Point2f topLeft;
    cv::Point2f bottomRight;

} object;

// container
std::vector <object> prev_objects;
std::vector <object> curr_objects;


float calculate_iou(cv::Point2f tl_a, cv::Point2f tl_b, cv::Point2f br_a, cv::Point2f br_b){

	float xA = max(tl_a.x,tl_b.x);
	float yA = max(tl_a.y,tl_b.y);
	float xB = min(br_a.x,br_b.x);
	float yB = min(br_a.y,br_b.y);

	float inter_area = (xB - xA + 1)*(yB - yA + 1);

	float boxAArea = (br_a.x - tl_a.x + 1)*(br_a.y - tl_a.y + 1);
	float boxBArea = (br_b.x - tl_b.x + 1)*(br_b.y - tl_b.y + 1);

	float iou = inter_area / (boxBArea + boxAArea - inter_area);

/*
	cout << "Object of INTEREST"<< endl;
	cout << "1. Intersectional Area:  " << inter_area << endl;
	cout << "2. Box A area: 		  " << boxAArea << endl;
	cout << "3. Box B area: 		  " << boxBArea << endl;
	cout << "4. Union Area: 		  " << (boxBArea + boxAArea - inter_area) << endl;
	cout << "5. IOU: 				  " << iou << endl;
*/
	return iou;
}


void subCallback(const neural_cam_ros::obstacleStack::ConstPtr& msg)
{
   //ROS_INFO("Hearing From: [%s]", msg->stack_name.c_str());

   int num_objects_det_curr = msg->stack_len;
   int num_objects_det_prev = (int) prev_objects.size();

   if(setFlag){   // first time

   		if(num_objects_det_curr > 0){   //first time seen objects

   			//start initial assignment
   			for(int i = 0; i < num_objects_det_curr; i++){

	   			object tempStorage;

	   			tempStorage.id = i+1;	//generate ID
	   			tempStorage.notFoundCounter = 0;
	   			tempStorage.topLeft.x = (float) msg->stack_obstacles[i].topleft.x;
	   			tempStorage.topLeft.y = (float) msg->stack_obstacles[i].topleft.y;
	   			tempStorage.bottomRight.x = (float) msg->stack_obstacles[i].bottomright.x;
	   			tempStorage.bottomRight.y = (float) msg->stack_obstacles[i].bottomright.y;

	   			prev_objects.push_back(tempStorage);
   			}

   			setFlag = false;
   		}

   }else{		// subsequent frame

   		/* -----> store objects into the curr_objects -----> */
   		object tempStorage;

   		for(int i = 0; i < num_objects_det_curr; i++){

   			tempStorage.topLeft.x = (float) msg->stack_obstacles[i].topleft.x;
   			tempStorage.topLeft.y = (float) msg->stack_obstacles[i].topleft.y;
   			tempStorage.bottomRight.x = (float) msg->stack_obstacles[i].bottomright.x;
   			tempStorage.bottomRight.y = (float) msg->stack_obstacles[i].bottomright.y;

   			curr_objects.push_back(tempStorage);
   		}
   		/* <----- store objects into the curr_objects <----- */


   		bool situation = true;
   		int max_mat_dim = max(num_objects_det_curr, num_objects_det_prev);

   		matrix<int> cost(max_mat_dim,max_mat_dim);

   		if(num_objects_det_curr <= num_objects_det_prev){       // Case 1: When detected objects in the current window less or equal to the previous detected

   			cout << "Case 1: curr <= prev" << endl;

	   		for(int i = 0; i < num_objects_det_prev; i++){

	   			for(int j = 0; j < num_objects_det_prev; j++){	//check which give the highest IOU

	   				if(j >= num_objects_det_curr){
	   					
	   					// missing detections in the current frame

	   					cost(i,j) = -1;							//assign dummy value

	   				}else{

	   					// calculate the IOU for each assignment 

	   					float iou_value = calculate_iou(prev_objects[i].topLeft, curr_objects[j].topLeft, prev_objects[i].bottomRight, curr_objects[j].bottomRight);

	   					cost(i,j) = (int) (iou_value*1000);

	   					//cout << "IOU: " << iou_value << endl;

	   				}
	   			}
	   		}

	   	}else{													// Case 2: When detected objects in the current window is strictly more than the previous detected

	   		situation = false;

	   		cout << "Case 2: prev <= curr" << endl;

	   		for(int i = 0; i < num_objects_det_curr; i++){ 			//looking at previous objects

		   		for(int j = 0; j < num_objects_det_curr; j++){

	   				if(i >= num_objects_det_prev){

	   					//cout << "Possible New Detections: " << (num_objects_det_curr - num_objects_det_prev) << endl;

	   					cost(i,j) = -1;				//assign dummy value

	   				}else{

		   				float iou_value = calculate_iou(prev_objects[i].topLeft, curr_objects[j].topLeft, prev_objects[i].bottomRight, curr_objects[j].bottomRight);

		   				cost(i,j) = (int) (iou_value*1000);

		   				//cout << "IOU: " << iou_value << endl;
		   			}

	   			}

	   		}

	   	}

	   	// This prints optimal assignments:  [2, 0, 1] which indicates that we should assign
    	// the person from the first row of the cost matrix to job 2, the middle row person to
    	// job 0, and the bottom row person to job 1.

	   	std::vector<long> assignment = dlib::max_cost_assignment(cost);
    	for (unsigned int i = 0; i < assignment.size(); i++){

        	cout << "Assignment: " << (int) assignment[i] << std::endl;

        	int assignment_to = (int) assignment[i];

        	if(situation){

        		// if current detected objects less than the previous detected ones, assign the current detected ones with previous id
        		// mark the rest as undetected for this current iteration

        		if(assignment_to >= num_objects_det_curr) {		// assigned to hoax object

        			// mark as undetected for this round

        			// TODO:

        			if(prev_objects[i].notFoundCounter < 5){ 	//assess if the previous item need to be included into the current object list or not

        				//kalman assignment here:

        				prev_objects[i].notFoundCounter++;
        				curr_objects.push_back(prev_objects[i]);
        			}

        		}else{

        			curr_objects[assignment_to].id = prev_objects[i].id;
        		}

        	}else{

        	 	if(i >= num_objects_det_prev){

        	 		//mark as new objects detected (generate new id/kalman filter)

        	 		// TODO:

        	 		curr_objects[assignment_to].id = 19992;
        	 		curr_objects[assignment_to].notFoundCounter = 0;

        	 		//insert new kalman filter here

        	 	}else{

        	 		curr_objects[assignment_to].id = prev_objects[i].id;
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