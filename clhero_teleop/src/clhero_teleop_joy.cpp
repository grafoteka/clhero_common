//=====================================================================
//  Author: Jorge De León Rivas
//  File: clhero_teleop_joy
//  Version: 1.0
//  Description: 
//  Paquete de teleoperación del CLHeRo. 
//  	Lee las flechas del pad (1/0/-1) y envía la orden de:
//  	- axes 5 > 0 =  avance, 
//  	- axes 5 < 0 = retroceso,
//  	- axes 4 > 0 = giro izquierda, 
//		- axes 4 < 0 = giro derecha.
//  Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <clhero_gait_controller/GaitPatternControl.h>

//----------------------------------------------------
//    Defines
//----------------------------------------------------


//----------------------------------------------------
//    Global Variables
//----------------------------------------------------
ros::ServiceClient client;
int move_command = 0;	// 1 = avance; 2 = izquierda; -2 = derecha; 0 = paro
int move_command_old = 0;

//----------------------------------------------------
//    Callbacks
//----------------------------------------------------
void joystick_cb(const sensor_msgs::Joy::ConstPtr& joy)
{
	// Avance: axes 5 = 1
	if((joy->axes[4] == 0) && (joy->axes[5] == 1) && (joy->buttons[3] == 0) && (joy->buttons[1] == 0))
	{
		move_command = 1;
	}

	// Derecha: axes 4 = 1
	else if((joy->axes[4] == -1) && (joy->axes[5] == 0) && (joy->buttons[3] == 0) && (joy->buttons[1] == 0))
	{
		move_command = -2;
	}
	// Izquierda: axes 4 = -1
	else if((joy->axes[4] == 1) && (joy->axes[5] == 0) && (joy->buttons[3] == 0) && (joy->buttons[1] == 0))
	{
		move_command = 2;
	}
	// Stand UP
	else if((joy->axes[4] == 0) && (joy->axes[5] == 0) && (joy->buttons[3] == 1) && (joy->buttons[1] == 0))
	{
		move_command = 10;
	}
	// Down
	else if((joy->axes[4] == 0) && (joy->axes[5] == 0) && (joy->buttons[1] == 1) && (joy->buttons[3] == 0))
	{
		move_command = -10;
	}
	else
		move_command = 0;
}

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main(int argc, char **argv)
{

	//----------------------------------------------------
	//    ROS starting statements
	//----------------------------------------------------

	ros::init(argc, argv, "clhero_teleop_joy");
	ros::NodeHandle nh;

	ros::Subscriber joystick = nh.subscribe("/joy", 1, joystick_cb);
	client = nh.serviceClient<clhero_gait_controller::GaitPatternControl>("/clhero_gait_control/gait_pattern_control");

	//ros::Publisher move_command_pub = nh.advertise("", 1);

	clhero_gait_controller::GaitPatternControl msg;


	while(ros::ok())
	{
		if(move_command != move_command_old)
		{
			// Parar movimiento
			if(move_command == 0)
			{
				ROS_INFO("Stop command");
				//msg.request.order = "stop";
				if(move_command_old == -2)
				{
					msg.request.pattern_name="turn_right_tripod";
					msg.request.order="stop";
				}

				else if(move_command_old == 2)
				{
					msg.request.pattern_name = "turn_left_tripod";
					msg.request.order = "stop";
				}

				else if(move_command_old == 1)
				{
					msg.request.pattern_name = "alternating_tripod";
					msg.request.order = "stop";
				}
				else if(move_command_old == 10)
				{
					msg.request.pattern_name = "stand_up";
					msg.request.order = "stop";
				}

				else if(move_command_old == -10)
				{
					msg.request.pattern_name = "get_down";
					msg.request.order = "stop";
				}
			}

			// Avanzar
			else if(move_command == 1)
			{
				ROS_INFO("Forward command");
				msg.request.pattern_name = "alternating_tripod";
				msg.request.order = "start";
				/*if(move_command_old == 0)
				{
					msg.request.pattern_name = "alternating_tripod";
					msg.request.order = "start";
				}

				else if(move_command_old == 2)
				{
					//msg.request.pattern_name("turn_left_tripod");
					//msg.request.order("stop");
					// Faltaría enviar mensaje aquí
					msg.request.pattern_name = "alternating_tripod";
					msg.request.order = "start";
				}

				else if(move_command_old == -2)
				{
					//msg.request.pattern_name("turn_right_tripod");
					//msg.request.order("stop");
					// Faltaría enviar mensaje aquí
					msg.request.pattern_name = "alternating_tripod";
					msg.request.order = "start";
				}*/
			}

			// Giro derecha
			else if(move_command == -2)
			{
				ROS_INFO("Turn right command");
				msg.request.pattern_name = "turn_right_tripod";
				msg.request.order = "start";
				/*if(move_command_old == 0)
				{
					msg.request.pattern_name = "turn_right_tripod";
					msg.request.order = "start";
				}

				else if(move_command_old == 2)
				{
					//msg.request.pattern_name("turn_left_tripod");
					//msg.request.order("stop");
					// Faltaría enviar mensaje aquí
					msg.request.pattern_name = "turn_right_tripod";
					msg.request.order = "start";
				}

				else if(move_command_old == 1)
				{
					//msg.request.pattern_name("alternating_tripod");
					//msg.request.order("stop");
					// Faltaría enviar mensaje aquí
					msg.request.pattern_name = "turn_right_tripod";
					msg.request.order = "start";
				}*/
			}

			// Giro izquierda
			else if(move_command == 2)
			{
				ROS_INFO("Turn left command");
				msg.request.pattern_name = "turn_left_tripod";
				msg.request.order = "start";
				/*if(move_command_old == 0)
				{
					msg.request.pattern_name = "turn_left_tripod";
					msg.request.order = "start";
				}

				else if(move_command_old == -2)
				{
					//msg.request.pattern_name("turn_right_tripod");
					//msg.request.order("stop");
					// Faltaría enviar mensaje aquí
					msg.request.pattern_name = "turn_left_tripod";
					msg.request.order = "start";
				}

				else if(move_command_old == 1)
				{
					//msg.request.pattern_name("alternating_tripod");
					//msg.request.order("stop");
					// Faltaría enviar mensaje aquí
					msg.request.pattern_name = "turn_left_tripod";
					msg.request.order = "start";
				}*/
			}

			// Stand UP
			else if(move_command == 10)
			{
				ROS_INFO("Stand UP");
				msg.request.pattern_name = "stand_up";
				msg.request.order = "start";
			}

			// Get down
			else if(move_command == -10)
			{
				ROS_INFO("Get Down");
				msg.request.pattern_name = "get_down";
				msg.request.order = "start";
			}

			else
			{
				ROS_INFO("Stop command");
				msg.request.order = "stop";
			}

			client.call(msg);
			msg.request.pattern_name.clear();
			msg.request.args.clear();
			msg.request.order.clear();
			move_command_old = move_command;
		}

		/*if(move_command == move_command_old)
		{
			//ROS_INFO("Entro alternativa");
			msg.request.order = "stop";
			client.call(msg);
			msg.request.order.clear();
		}*/

		ros::spinOnce();
	}
}

