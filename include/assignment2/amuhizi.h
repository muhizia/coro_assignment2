/*******************************************************************************************************************
*
*   Assignment 2: Implementation of the divide-and-conquer go-to-position algorithm (goto1) 
*                 and the MIMO go-to-position algorithm (goto2)
*
*   This is the interface file.
*   For documentation, please see the application file
*
*   David Vernon
*   17 February 2021
*
*   Audit Trail
*   -----------
*
*
*
*******************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h> // for turtle1/teleport_absolute service
#include <turtlesim/SetPen.h>           // for turtle1/set_pen service
#include <std_srvs/Empty.h>             // for reset and clear services
#include <geometry_msgs/Twist.h>        // For geometry_msgs::Twist 
#include <iomanip>                      // for std::setprecision and std::fixed
#include <turtlesim/Spawn.h>
using namespace std;

#define ROS_PACKAGE_NAME    "assignment2"
#define MAX_FILENAME_LENGTH 200


/* Callback function, executed each time a new pose message arrives */

void poseMessageReceived(const turtlesim::Pose& msg);


void display_error_and_exit(char error_message[]);
void prompt_and_exit(int status);
void prompt_and_continue();
void print_message_to_file(FILE *fp, char message[]);
void rotateTutle();
void devideAndConquer(ros::Publisher  pub, double x_g, double y_g, double theta_g);
void MeMo(ros::Publisher  pub, double x_g, double y_g, double theta_g);
void getDirection(double *direction);