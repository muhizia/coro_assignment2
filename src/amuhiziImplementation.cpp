/*******************************************************************************************************************
*   
*  Assignment 2: Implementation of the divide-and-conquer go-to-position algorithm (goto1) 
*                and the MIMO go-to-position algorithm (goto2)
*
*   This is the implementation file.
*   For documentation, please see the application file
*
*   David Vernon
*   7 August 2020
*
*   Audit Trail
*   -----------
*     - Added MeMo() to allow the turtle moving and turning at the sametime.
*     - Added devideAndConquer() this funtion allow the turtle to turn first and move after
*     - Added getDirection() to get the angle of the turtle Omega.
*******************************************************************************************************************/

#include <assignment2/amuhizi.h> 

  
/* global variables with the current turtle pose */

extern float         current_x; 
extern float         current_y; 
extern float         current_theta;

/* Callback function, executed each time a new pose message arrives */

void poseMessageReceived(const turtlesim::Pose& msg) {
  bool debug = false;

   if (debug) {ROS_INFO_STREAM(std::setprecision(2) << std::fixed <<
	                       "position=(" << msg.x << "," << msg.y << ")" <<
		               " direction=" << msg.theta);
   }
   
   current_x     = msg.x;
   current_y     = msg.y;
   current_theta = msg.theta;
}

/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/


void display_error_and_exit(char error_message[]) {
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(1);
}

void prompt_and_exit(int status) {
   printf("Press any key to terminate the program ... \n");
   getchar();
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}


void print_message_to_file(FILE *fp, char message[]) {
   fprintf(fp,"The message is: %s\n", message);
}
/** 
 * devide and conquer algorithm
 * Turning the turtle before moving
 * 
*/
void devideAndConquer(ros::Publisher  pub, double x_g, double y_g, double theta_g){
   ros::Rate _rate(1);
   double dx, dy, currentX, currentY, currentTheta;
   double erro_pos, erro_h;
   geometry_msgs::Twist _msg;
   srand(time(NULL));
   bool is_start = true;
   double Kpp = 0.5;
   double Kph = 0.2;
   ROS_INFO("Moving error pos = %.2f, erro header %.2f\n", erro_pos, erro_h);
   ROS_INFO("current x = %.2f, current y %.2f\n", currentX, currentY);

   while (is_start || abs(erro_pos) > 1e-2){
      is_start       = false;
      currentX       = current_x;
      currentY       = current_y;
      currentTheta   = current_theta;
      dx             = x_g-currentX;
      dy             = y_g-currentY;
      erro_pos       = sqrt(dx*dx + dy*dy);
      erro_h         = atan2(dy, dx) - currentTheta;

      ROS_INFO("Moving theta_g = %.2f, currentTheta %.2f\n", current_theta, currentTheta);
      // ROS_INFO("Moving error pos = %.2f, erro header %.2f\n", erro_pos, erro_h);
      // ROS_INFO("current Dx = %.2f, Dy %.2f\n", dx, dy);
      // ROS_INFO("current x = %.2f, current y %.2f\n", currentX, currentY);
      

      if (abs(erro_h)>0.000872665){
         // setting the angular velocity
         _msg.linear.x = 0;
         _msg.linear.y = 0;
         _msg.linear.z = 0;

         _msg.angular.x = 0;
         _msg.angular.y = 0;
         _msg.angular.z = Kph*erro_h;
         // ROS_INFO("angular z = %.2f error h = %.2f\n", Kph*erro_h, erro_h);
         // ROS_INFO("============== Turning =========");
      }else{
         // seting values for the linear velocity
         _msg.linear.x = Kpp*erro_pos;
         _msg.linear.y = 0;
         _msg.linear.z = 0;

         _msg.angular.x = 0;
         _msg.angular.y = 0;
         _msg.angular.z = 0;
         // ROS_INFO("Linear z = %.2f error position = %.2f\n", Kpp*erro_pos, erro_pos);
         // ROS_INFO("============= Moving ==========");
      }
      //   ROS_INFO("Moving Linear.x = %.2f, angular.z %.2f\n", _msg.linear.x, _msg.angular.x);
      // Publishing the angular velocity and linear velocity
      pub.publish(_msg);
      ros::spinOnce();
      _rate.sleep();
   }
   ROS_INFO("Finished.....\n");

   //Publishing the value to stop the turtle.
   _msg.linear.x = 0;
   pub.publish(_msg);
}

void MeMo(ros::Publisher  pub, double x_g, double y_g, double theta_g){
   ros::Rate _rate(1);
   // variable that will be used
   double dx, dy, currentX, currentY, currentTheta;
   double erro_pos, erro_h;
   geometry_msgs::Twist _msg;
   srand(time(NULL));
   bool is_start = true;
   double Kpp = 0.1;
   double Kph = 0.3;
   // ROS_INFO("Moving error pos = %.2f, erro header %.2f\n", erro_pos, erro_h);
   // ROS_INFO("current x = %.2f, current y %.2f\n", currentX, currentY);

   /** 
    * 
    * setting angular velocity x and Y to zero
    * linear y, z to zero
    * 
   */
   _msg.angular.x = 0;
   _msg.angular.y = 0;
   _msg.linear.y  = 0;
   _msg.linear.z  = 0;
   while (is_start || abs(erro_pos) > 1e-2){
      is_start       = false;
      currentX       = current_x;
      currentY       = current_y;
      currentTheta   = current_theta;
      // getDirection(&currentTheta);
      dx             = x_g-currentX;
      dy             = y_g-currentY;
      erro_pos       = sqrt(dx*dx + dy*dy);
      erro_h         = atan2(dy, dx) - currentTheta;

      // ROS_INFO("Moving theta_g = %.2f, currentTheta %.2f\n", theta_g, currentTheta);
      // ROS_INFO("Moving error pos = %.2f, erro header %.2f\n", erro_pos, erro_h);
      // ROS_INFO("current Dx = %.2f, Dy %.2f\n", dx, dy);
      // ROS_INFO("current x = %.2f, current y %.2f\n", currentX, currentY);
      
      
      _msg.angular.z = Kph*erro_h;
      _msg.linear.x  = Kpp*erro_pos;
      

   
      // ROS_INFO("angular z = %.2f error h = %.2f\n", Kph*erro_h, erro_h);
      // ROS_INFO("Linear x = %.2f error position = %.2f\n", Kpp*erro_pos, erro_pos);
      // ROS_INFO("============= Moving ==========");

      //publishing the velocity
      pub.publish(_msg);
      ros::spinOnce();
      _rate.sleep();
   }
   // ROS_INFO("End.....\n");
   //ending the program buy setting the velocity to zero
   _msg.linear.x = 0;
   pub.publish(_msg);
}