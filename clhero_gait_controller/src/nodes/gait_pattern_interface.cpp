//=====================================================================
//	Author:	Raúl Cebolla Arroyo
//	File:
//	Version:
//	Description:
//	Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <clhero_gait_controller/GaitPatternControl.h>
#include <clhero_gait_controller/PatternCommand.h>
#include <clhero_gait_controller/ChangeGaitPattern.h>
#include <clhero_gait_controller/RegisterGaitPattern.h>
#include <vector>
#include <string>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

#define CURRENT_GP_PARAM_NAMESPACE "/clhero/gait_patterns/current"
#define REGISTERED_GP_PARAM_NAMESPACE "/clhero/gait_patterns/registered"

//----------------------------------------------------
//    Class definitions
//----------------------------------------------------

//--None--

//----------------------------------------------------
//    Global variables
//----------------------------------------------------

//Global variable that sets the node to debug mode
int debug_mode;

//Client for the change gait pattern service
ros::ServiceClient change_gp_client;

//Publisher for the msg to control the gait patterns
ros::Publisher pattern_command_pub;

//Active gait_pattern
std::string current_pattern = "None";

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function which checks and sets the debug mode
bool checkDebugMode(int argc, char** argv){

  debug_mode = 0;

  for(int i = 0; i < argc; i++){
    if(!strcmp(argv[i], "debug")){
      ROS_INFO("Entering debug mode.");
      debug_mode = 1;
      return true;
    }
  }

  return false;
}

//Function that checks if a name is in the registered gait patterns list
bool isPatternRegistered (std::string pattern_name){
  
  //Gets the registered patterns name list
  std::vector<std::string> reg_names_list;
  ros::NodeHandle nh;

  nh.getParam(REGISTERED_GP_PARAM_NAMESPACE, reg_names_list);

  //Checks if the name is in the list
  for(int i = 0; i<reg_names_list.size(); i++){
      
      //if the name matches
      if(!reg_names_list[i].compare(pattern_name)){
        return true;
      }
  }

  //The name is not in the list
  return false;
}

//Function that builds the pattern command msg
clhero_gait_controller::PatternCommand buildPatternCommandMsg (clhero_gait_controller::GaitPatternControl::Request &req){
  clhero_gait_controller::PatternCommand msg;
  msg.pattern_name = req.pattern_name;
  msg.order = req.order;
  msg.args = req.args;
  return msg;
}

//Function that handles the gait pattern change service
bool gaitPatternControlCallback (clhero_gait_controller::GaitPatternControl::Request &req, 
                                 clhero_gait_controller::GaitPatternControl::Response &res){

  std::string order = req.order;
  clhero_gait_controller::PatternCommand msg;
  ros::NodeHandle nh;

  //---------------------------------------------------
  //  start order
  //---------------------------------------------------

  if(!order.compare("start")){
    //checks if the pattern name is in the registered patterns'list
    if(isPatternRegistered(req.pattern_name)){

      //Requests a pattern change at the command interface
      clhero_gait_controller::ChangeGaitPattern change_req;
      change_req.request.name = req.pattern_name;
      
      if(!change_gp_client.call(change_req)){
        ROS_ERROR("Couldn't call command interface for change_gait_pattern service");
        res.ans = 4;
        return true;
      }

      if(change_req.response.ans){
        ROS_ERROR("Couldn't change gait pattern at command interface");
        res.ans = 4;
        return true;
      }

      //Sets the name as current
      current_pattern = req.pattern_name;
      nh.setParam(CURRENT_GP_PARAM_NAMESPACE, current_pattern);
      
      //if the name is in the list, bulids the msg
      msg = buildPatternCommandMsg(req);
      //and publishes it
      pattern_command_pub.publish(msg);

      res.ans = 0;
      return true;

    }else{
      //The name is not registered
      res.ans = 2;
      return true;
    }  
  }

  //---------------------------------------------------
  //  pause order
  //---------------------------------------------------

  if(!order.compare("pause")){
    
    //Checks if the name matches the current gait pattern
    if(!current_pattern.compare(req.pattern_name)){

      //If matches, builds the msg
      msg = buildPatternCommandMsg(req);
      //and publishes it
      pattern_command_pub.publish(msg);

      res.ans = 0;
      return true;

    }else{
      //If not matches an error shall be given
      res.ans = 3;
      return true;
    }
  }

  //---------------------------------------------------
  //  continue order
  //---------------------------------------------------

  if(!order.compare("continue")){
    
    //Checks if the name matches the current gait pattern
    if(!current_pattern.compare(req.pattern_name)){

      //If matches, builds the msg
      msg = buildPatternCommandMsg(req);
      //and publishes it
      pattern_command_pub.publish(msg);

      res.ans = 0;
      return true;

    }else{
      //If not matches an error shall be given
      res.ans = 3;
      return true;
    }
  }

  //---------------------------------------------------
  //  force_state order
  //---------------------------------------------------

  if(!order.compare("force_state")){
    
    //Checks if the name matches the current gait pattern
    if(!current_pattern.compare(req.pattern_name)){

      //If matches, builds the msg
      msg = buildPatternCommandMsg(req);
      //and publishes it
      pattern_command_pub.publish(msg);

      res.ans = 0;
      return true;

    }else{
      //If not matches an error shall be given
      res.ans = 3;
      return true;
    }
  }

  //---------------------------------------------------
  //  stop order
  //---------------------------------------------------

  if(!order.compare("stop")){
    
    //Checks if the name matches the current gait pattern
    if(!current_pattern.compare(req.pattern_name)){

      //If matches, builds the msg
      msg = buildPatternCommandMsg(req);
      //and publishes it
      pattern_command_pub.publish(msg);

      res.ans = 0;
      return true;

    }else{
      //If not matches an error shall be given
      res.ans = 3;
      return true;
    }
  }

  //---------------------------------------------------
  //  update_args order
  //---------------------------------------------------

  if(!order.compare("update_args")){
    
    //Checks if the name matches the current gait pattern
    if(!current_pattern.compare(req.pattern_name)){

      //If matches, builds the msg
      msg = buildPatternCommandMsg(req);
      //and publishes it
      pattern_command_pub.publish(msg);

      res.ans = 0;
      return true;

    }else{
      //If not matches an error shall be given
      res.ans = 3;
      return true;
    }
  }

  //---------------------------------------------------
  //  order not recognized
  //---------------------------------------------------

  res.ans = 1;

  return true;
}

//Callback for registering a new gait pattern
bool gaitPatternRegisterCallback (clhero_gait_controller::RegisterGaitPattern::Request &req, 
                                 clhero_gait_controller::RegisterGaitPattern::Response &res){

  //Node handle
  ros::NodeHandle nh;

  //List with the registered gait patterns
  std::vector<std::string> registered_gp;

  //Checks if the parameter exists in the parameter server and in that case takes it.
  if(nh.hasParam(REGISTERED_GP_PARAM_NAMESPACE)){
    nh.getParam(REGISTERED_GP_PARAM_NAMESPACE, registered_gp);
  }else{
    res.ans = false;
    return true;
  }

  //Checks if the pattern name already exists 
  for(int i = 0; i < registered_gp.size(); i++){
    if(registered_gp[i].compare(req.pattern_name) == 0){
      //The name has been already registered
      res.ans = false;
      return true;
    }
  }

  //Add the new gaitpattern name
  registered_gp.push_back(req.pattern_name);

  //Upload the new parameter
  nh.setParam(REGISTERED_GP_PARAM_NAMESPACE, registered_gp);

  res.ans = true;

  return true;

}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main(int argc, char **argv){

  //----------------------------------------------------
  //    ROS starting statements
  //----------------------------------------------------

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  checkDebugMode(argc, argv);

  //Creates the parameter for the registered gait patterns list
  std::vector<std::string> empty_list;
  nh.setParam(REGISTERED_GP_PARAM_NAMESPACE, empty_list);

  //Creation of the client for changing the gait_pattern
  change_gp_client = nh.serviceClient<clhero_gait_controller::ChangeGaitPattern>("change_gait_pattern");

  //Creation of the msg publisher
  pattern_command_pub = nh.advertise<clhero_gait_controller::PatternCommand>("pattern_command", 1000);

  //Creation of the server for the control service
  ros::ServiceServer gp_control_srv = nh.advertiseService("gait_pattern_control", gaitPatternControlCallback);
  ros::ServiceServer register_gait_pattern_srv = nh.advertiseService("register_gait_pattern", gaitPatternRegisterCallback);

  //----------------------------------------------------
  //    Core loop of the node
  //----------------------------------------------------
  ros::spin();

  return 0;

}
