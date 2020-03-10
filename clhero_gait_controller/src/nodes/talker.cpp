#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
//#include <iostream>
//#include "std_msgs/Float32.h"
//#include "std_msgs/Float64.h"
//#include <sstream>
//#include <string>
//#include <ros/console.h>
//#include <stdlib.h>
//#include <geometry_msgs/Twist.h>
//#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

//#include <sstream>

float joint1_state = 0;
float joint2_state = 0;
float joint3_state = 0;
float joint4_state = 0;
float joint5_state = 0;
float angle =1.0;
double prueba;

sensor_msgs::JointState lectura;

//_Float64 angle = 0.0;
/**
 * Suscripción a los tópicos de posición de las articulaciones del robot
 */
//void cmd_cb(const sensor_msgs::JointState::ConstPtr robot_igus_final)
  void cmd_cb(const sensor_msgs::JointState igus_5DOF_SV)
 {
 //sensor_msgs::JointState lectura = *igus_5DOF_SV;
 ROS_INFO("antes") ;
 lectura = igus_5DOF_SV;
 //angle = 2.0;
 //lectura.position[0] = igus_5DOF_SV->position[0];

 ROS_INFO("Valor = %lf",lectura.position.at(0));
 //ROS_INFO("Valor = %d", lectura.position.size());
 //angle = 1.0;
 }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "articulaciones");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/joint_states",1000, cmd_cb);
  //ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", &cmd_cb);


  ros::Publisher joint1 = n.advertise<std_msgs::Float64>("/robot_igus_final/joint1_position_controller/command", 1000);
  /*ros::Publisher joint2 = n.advertise<std_msgs::Float64>("/robot_igus_final/joint2_position_controller/command", 1000);
  ros::Publisher joint3 = n.advertise<std_msgs::Float64>("/robot_igus_final/joint3_position_controller/command", 1000);
  ros::Publisher joint4 = n.advertise<std_msgs::Float64>("/robot_igus_final/joint4_position_controller/command", 1000);
  ros::Publisher joint5 = n.advertise<std_msgs::Float64>("/robot_igus_final/joint5_position_controller/command", 1000);

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  */
  ros::Rate loop_rate(10);


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  std_msgs::Float64 dato;

  dato.data = 0;

  ROS_INFO("Inicio");
  while (ros::ok())
  {
    //ROS_INFO("while");
    //ROS_INFO(">> %f",angle);
    joint1.publish(dato);
    /*joint2.publish(0.3);
    joint3.publish(0.3);
    joint4.publish(1.3);
    joint5.publish(0.0);*/


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
