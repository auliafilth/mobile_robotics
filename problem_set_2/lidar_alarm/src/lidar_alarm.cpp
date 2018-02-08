// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <iostream>
#include <cmath>


const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_= 0.0;
double angle_max_= 0.0;
double angle_increment_= 0.0;
double range_min_ =  0.0;
double range_max_ =  0.0;

bool laser_alarm_=false;
int laser_counter = 0; // for our for loop
int safe_min_index = 0; // init ping index for our min angle
int safe_max_index = 0; // init ping index for our max angle
double safe_angle_ = 1.4; // we used to use 150 and 530

double path_length = 0.5; // length of the clear path to check
double path_width = 0.25; // width of the clear path to check
int bottom_corner_index = 0; //bottom corner of the box (right of bot)
int top_corner_index = 0; // top corner of the box (left of bot)
int bottom_right_angle_index = 0; // init right angle index for theta calculations below
int top_right_angle_index = 0;
double theta = 0.0; // theta will constantly change, 

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index_ = %d",ping_index_);
	safe_min_index = (int) ping_index_ - (safe_angle_/angle_increment_);
	//ROS_INFO("LIDAR setup: safe_min_index = %d",safe_min_index);
	safe_max_index = (int) ping_index_ + (safe_angle_/angle_increment_);
	//ROS_INFO("LIDAR setup: safe_max_index = %d",safe_max_index);

	bottom_corner_index = (int) ping_index_ - (atan2(path_width/2,path_length)/angle_increment_);  // order of width and height might be wrong
        top_corner_index = (int) ping_index_ + (atan2(path_width/2,path_length)/angle_increment_);
        bottom_right_angle_index = (int) ping_index_ - safe_angle_/angle_increment_;
        top_right_angle_index = (int) ping_index_ + safe_angle_/angle_increment_;

        //ROS_INFO("anglemin = %f",angle_min_);
        //ROS_INFO("angle increment = %f",angle_increment_);
        //ROS_INFO("rangemin = %f",range_min_);
        //ROS_INFO("rangemax = %f",range_max_);
        
    }


   laser_counter = 0;
   for(int i=safe_min_index; i<=safe_max_index; i++){

       ping_dist_in_front_ = laser_scan.ranges[i];
       ROS_INFO("ping dist %d in front = %f", i, ping_dist_in_front_);
       if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
   	       laser_counter++;
       }
   }
   if (laser_counter > 0){
   	   laser_alarm_=true;
   	   ROS_WARN("DANGER, WILL ROBINSON!!");
   }
   else {
       laser_alarm_=false;
   }

/**
// the new way of sensing a wall, at least my attempt
// This didn't work. The way I'm calculating my safe path walls isn't right. I keep getting an inf space.
   for(int i=safe_min_index; i<=bottom_corner_index; i++){
      ping_dist_in_front_ = laser_scan.ranges[i];
      ROS_INFO("ping %d = %f, and safe is %f", i, ping_dist_in_front_, path_width/(2*abs(cos(theta))));
      theta = ping_index_*angle_increment_ - bottom_right_angle_index;
      if (ping_dist_in_front_<(path_width/(2*abs(cos(theta))))) {
         laser_counter++;
       }
   }
   for(int i=bottom_corner_index; i<=ping_index_; i++){
      ping_dist_in_front_ = laser_scan.ranges[i];
      ROS_INFO("ping %d = %f, and safe is %f", i, ping_dist_in_front_, path_length/(abs(sin(theta))));
      theta = ping_index_*angle_increment_ - bottom_right_angle_index;
      if (ping_dist_in_front_<(path_length/(abs(sin(theta))))) {
         laser_counter++;
       }
   }
   for(int i=ping_index_; i<=top_corner_index; i++){
      ping_dist_in_front_ = laser_scan.ranges[i];
      ROS_INFO("ping %d = %f, and safe is %f", i, ping_dist_in_front_, path_length/(abs(sin(theta - 3.1415/2))));
      theta = ping_index_*angle_increment_ - bottom_right_angle_index;
      if (ping_dist_in_front_<(path_length/(abs(sin(theta - 3.1415/2))))) {
         laser_counter++;
       }
   }
   for(int i=top_corner_index; i<=safe_max_index; i++){
      ping_dist_in_front_ = laser_scan.ranges[i];
      ROS_INFO("ping %d = %f, and safe is %f", i, ping_dist_in_front_, path_width/(2*abs(cos(3.1415 - theta))) );
      theta = ping_index_*angle_increment_ - bottom_right_angle_index;
      if (ping_dist_in_front_<(path_width/(2*abs(cos(3.1415 - theta))))) {
         laser_counter++;
       }
   }
   if (laser_counter > 0){
   	   laser_alarm_=true;
   	   ROS_WARN("DANGER, WILL ROBINSON!!");
   }
   else {
       laser_alarm_=false;
   }
**/

   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

