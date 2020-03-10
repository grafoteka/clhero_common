#include <ros/ros.h>
#include <clhero_gait_controller/LegState.h>
#include <sensor_msgs/JointState.h>


//----------------------------------------------------
//    Global Variables
//----------------------------------------------------
std::vector<double>legs_position = {0, 0, 0, 0, 0, 0};
std::vector<double>legs_velocity = {0, 0, 0, 0, 0, 0};
std::vector<double>legs_effort   = {0, 0, 0, 0, 0, 0};

//----------------------------------------------------
//    Defines
//----------------------------------------------------
#define LEG_NUMBER 6 //Number of legs of the robot

//----------------------------------------------------
//    Callbacks
//----------------------------------------------------

//Callback for legs state
void legsStateCallback (const clhero_gait_controller::LegState::ConstPtr& msg){

    double position, velocity, effort;

    legs_position.clear();
    legs_velocity.clear();
    legs_effort.clear();

    //For each leg
    for(int i = 0; i < LEG_NUMBER; i++){
        legs_position.push_back(msg->pos.at(i));
        legs_velocity.push_back(msg->vel.at(i));
        legs_effort.push_back(msg->torq.at(i));
    }


}

/*void legCommandCallback (const clhero_gait_controller::LegCommand::ConstPtr& msg){

        Stopwatch watch;

        watch.start();
        com_man->evaluateNewCommand(msg);
        com_man->commandAllMotors();
        watch.stop();

        reg->write("legCommandCallback", watch.get_interval());

        return;
}*/


//legs_state_pub = nh.advertise<clhero_gait_controller::LegState>("legs_state", 10);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clhero_joint_states");
  ros::NodeHandle nh;

  sensor_msgs::JointState clhero_joint_states;

  //Topics subscription
  ros::Subscriber legs_state_sub = nh.subscribe("/clhero_gait_control/legs_state", 1, legsStateCallback);

  // Topics publisher
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/clhero/joint_states", 1);

  clhero_joint_states.header.stamp = ros::Time::now();
  clhero_joint_states.name.resize(6);
  clhero_joint_states.name[0] = "pata_1_to_base_link_joint";
  clhero_joint_states.name[1] = "pata_2_to_base_link_joint";
  clhero_joint_states.name[2] = "pata_3_to_base_link_joint";
  clhero_joint_states.name[3] = "pata_4_to_base_link_joint";
  clhero_joint_states.name[4] = "pata_5_to_base_link_joint";
  clhero_joint_states.name[5] = "pata_6_to_base_link_joint";

  clhero_joint_states.position.resize(6);
  clhero_joint_states.velocity.resize(6);
  clhero_joint_states.effort.resize(6);

  for(int i = 0; i < LEG_NUMBER; i++){
      clhero_joint_states.position[i] = 0.0;
      clhero_joint_states.velocity[i] = 0.0;
      clhero_joint_states.effort[i] = 0.0;
  }

  ROS_INFO("Hello world!");

  while(ros::ok()){

      clhero_joint_states.header.stamp = ros::Time::now();

      for(int i = 0; i < LEG_NUMBER; i++){
          clhero_joint_states.position.push_back(legs_position.at(i));
          clhero_joint_states.velocity.push_back(legs_velocity.at(i));
          clhero_joint_states.effort.push_back(legs_effort.at(i));
      }

      joint_state_pub.publish(clhero_joint_states);
      clhero_joint_states.position.clear();
      clhero_joint_states.velocity.clear();
      clhero_joint_states.effort.clear();
      //clhero_joint_states.clear();

      ros::spinOnce();
  }

  return 0;
}
