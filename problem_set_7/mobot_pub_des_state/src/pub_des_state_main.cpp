#include "pub_des_state.h"

bool g_lidar_alarm=false; // global var for lidar alarm

// This is what sets the lidar alarm global variable, responds to alarm_msg
void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!"); 
  }
} 

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;
    //instantiate a desired-state publisher object
    DesStatePublisher desStatePublisher(nh);

    ros::Subscriber alarm_subscriber = nh.subscribe("lidar_alarm",1,alarmCallback);  // Yo is it okay if I plop a subscriber in here?

    //dt is set in header file pub_des_state.h    
    ros::Rate looprate(1 / dt); //timer for fixed publication rate
    desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
    //put some points in the path queue--hard coded here
    desStatePublisher.append_path_queue(5.0,0.0,0.0);
    desStatePublisher.append_path_queue(0.0,0.0,0.0);
    
    // main loop; publish a desired state every iteration
    while (ros::ok()) {
        desStatePublisher.pub_next_state(g_lidar_alarm);
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
    }
}
