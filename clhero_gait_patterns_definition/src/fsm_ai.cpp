//=====================================================================
//  Author: Jorge De León Rivas
//  File: fsm_ai
//  Version: 1.0
//  Description: Modo de marcha que obtiene por argumentos 
//               constantemente la consigna de posición de las patas
//  Changelog:
//=====================================================================

//----------------------------------------------------
//    Includes
//----------------------------------------------------

#include <ros/ros.h>
#include <clhero_gait_controller/clhero.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>

//----------------------------------------------------
//    Defines
//----------------------------------------------------

//Declare the defines needed in the gait pattern

#define STATE_LOOP_RATE 5
#define PATTERN_NAME "fsm_ai"
#define STAND_UP_VEL 2.5
#define PI 3.14159265359
#define ANG_THR 0.17453292519943295 // 10[º]
#define LEGS_INITIAL_OFFSET 4.719	// Posicion inicial de las patas al estar en el suelo
#define LEG_NUMBER 6

//----------------------------------------------------
//    Global Variables
//----------------------------------------------------

//Tripods
const std::vector<int> tripod_1 = {1, 4, 5};
const std::vector<int> tripod_2 = {2, 3, 6};
const std::vector<int> all_legs = {1, 2, 3, 4, 5, 6};
std::vector<float> legs_position = {4.57, 4.57, 4.57, 4.57, 4.57, 4.57};

//Movement Parameters
std::unordered_map<std::string, double> movement_parameters;

//----------------------------------------------------
//    Functions
//----------------------------------------------------

//Function that updates the parameters
std::unordered_map<std::string, double> update_movement_parameters(std::map<std::string,std::string> args){

	//Gets each of the parameters
	
	//Leg 1
	try{
		movement_parameters["leg_1"] = std::stod(args.at("leg_1"));
		ROS_INFO("[fsm_ai]: Leg 1 position, setting to: %2f", movement_parameters["leg_1"]);

		//checks that the value is inside 1 turn (0-360, 0-2PI)
		if(movement_parameters["leg_1"] > 360){
			ROS_INFO("[GET PARAM leg_1]: Angle value exceeded, setting from %.2f to %.2f", movement_parameters["leg_1"], fmod(movement_parameters["leg_1"],2*PI));
			movement_parameters["leg_1"] = fmod(movement_parameters["leg_1"],2*PI);
		}

	}catch(std::out_of_range& oor){
		movement_parameters["leg_1"] = legs_position[0];
	}

	//Leg 2
	try{
		movement_parameters["leg_2"] = std::stod(args.at("leg_2"));
		ROS_INFO("[fsm_ai]: Leg 2 position, setting to: %2f", movement_parameters["leg_2"]);

		//checks that the value is inside 1 turn (0-360, 0-2PI)
		if(movement_parameters["leg_1"] > 360){
			ROS_INFO("[GET PARAM leg_1]: Angle value exceeded, setting from %.2f to %.2f", movement_parameters["leg_1"], fmod(movement_parameters["leg_1"],2*PI));
			movement_parameters["leg_1"] = fmod(movement_parameters["leg_1"],2*PI);
		}

	}catch(std::out_of_range& oor){
		movement_parameters["leg_2"] = legs_position[1];
	}

	//Leg 3
	try{
		movement_parameters["leg_3"] = std::stod(args.at("leg_3"));
		ROS_INFO("[fsm_ai]: Leg 3 position, setting to: %2f", movement_parameters["leg_3"]);

		//checks that the value is inside 1 turn (0-360, 0-2PI)
		if(movement_parameters["leg_1"] > 360){
			ROS_INFO("[GET PARAM leg_1]: Angle value exceeded, setting from %.2f to %.2f", movement_parameters["leg_1"], fmod(movement_parameters["leg_1"],2*PI));
			movement_parameters["leg_1"] = fmod(movement_parameters["leg_1"],2*PI);
		}

	}catch(std::out_of_range& oor){
		movement_parameters["leg_3"] = legs_position[2];
	}

	//Leg 4
	try{
		movement_parameters["leg_4"] = std::stod(args.at("leg_4"));
		ROS_INFO("[fsm_ai]: Leg 4 position, setting to: %2f", movement_parameters["leg_4"]);

		//checks that the value is inside 1 turn (0-360, 0-2PI)
		if(movement_parameters["leg_1"] > 360){
			ROS_INFO("[GET PARAM leg_1]: Angle value exceeded, setting from %.2f to %.2f", movement_parameters["leg_1"], fmod(movement_parameters["leg_1"],2*PI));
			movement_parameters["leg_1"] = fmod(movement_parameters["leg_1"],2*PI);
		}

	}catch(std::out_of_range& oor){
		movement_parameters["leg_4"] = legs_position[3];
	}

	//Leg 5
	try{
		movement_parameters["leg_5"] = std::stod(args.at("leg_5"));
		ROS_INFO("[fsm_ai]: Leg 5 position, setting to: %2f", movement_parameters["leg_5"]);

		//checks that the value is inside 1 turn (0-360, 0-2PI)
		if(movement_parameters["leg_1"] > 360){
			ROS_INFO("[GET PARAM leg_1]: Angle value exceeded, setting from %.2f to %.2f", movement_parameters["leg_1"], fmod(movement_parameters["leg_1"],2*PI));
			movement_parameters["leg_1"] = fmod(movement_parameters["leg_1"],2*PI);
		}

	}catch(std::out_of_range& oor){
		movement_parameters["leg_5"] = legs_position[4];
	}

	//Leg 6
	try{
		movement_parameters["leg_6"] = std::stod(args.at("leg_6"));
		ROS_INFO("[fsm_ai]: Leg 6 position, setting to: %2f", movement_parameters["leg_6"]);

		//checks that the value is inside 1 turn (0-360, 0-2PI)
		if(movement_parameters["leg_1"] > 360){
			ROS_INFO("[GET PARAM leg_1]: Angle value exceeded, setting from %.2f to %.2f", movement_parameters["leg_1"], fmod(movement_parameters["leg_1"],2*PI));
			movement_parameters["leg_1"] = fmod(movement_parameters["leg_1"],2*PI);
		}

	}catch(std::out_of_range& oor){
		movement_parameters["leg_6"] = legs_position[5];
	}

	return movement_parameters;

}



//----------------------------------------------------
//    States
//----------------------------------------------------

//Estado 1
void state_1 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	ROS_INFO("[FSM_AI]: Entered state 1");

	update_movement_parameters(clhr->getArgs());

	std::vector<float> state;	//Posicion de las patas
	int legs_in_position = 0;

	
	/*state = clhr->getLegsPosition();
	for(int i=0; i < LEG_NUMBER; i++){
		ROS_INFO("La posicion de la pata %d es: %.2f", i+1, state[i]);
	}*/
	
	//int consigna_alcanzada = 0; //Es lo mismo pero solo para 1 pata

	//ROS_INFO("He conseguido regresar");

	const double stand_up_vel = STAND_UP_VEL;
	double leg_1_consigna = movement_parameters["leg_1"]; //legs_position[0];
	double leg_2_consigna = movement_parameters["leg_2"]; //legs_position[1];
	double leg_3_consigna = movement_parameters["leg_3"]; //legs_position[2];
	double leg_4_consigna = movement_parameters["leg_4"]; //legs_position[3];
	double leg_5_consigna = movement_parameters["leg_5"]; //legs_position[4];
	double leg_6_consigna = movement_parameters["leg_6"]; //legs_position[5];

	std::vector<double> legs_consigna = {movement_parameters["leg_1"],
										 movement_parameters["leg_2"],
										 movement_parameters["leg_3"],
										 movement_parameters["leg_4"], 
										 movement_parameters["leg_5"],
										 movement_parameters["leg_6"]};

	std::vector<double> legs_consigna_old = {0, 0, 0, 0, 0, 0};

	// La accion de esta FSM es levantar al robot, así que asumimos que las patas no están en posicion
	clhr->setLegPosition(1, leg_1_consigna, stand_up_vel);
	clhr->setLegPosition(2, leg_2_consigna, stand_up_vel);
	clhr->setLegPosition(3, leg_3_consigna, stand_up_vel);
	clhr->setLegPosition(4, leg_4_consigna, stand_up_vel);
	clhr->setLegPosition(5, leg_5_consigna, stand_up_vel);
	clhr->setLegPosition(6, leg_6_consigna, stand_up_vel);
	clhr->sendCommands();

	bool consigna_alcanzada = false;
	//As for example
	// for(int i=0; i<6; i++){
	// 	clhr->setLegPosition(i+1, 90);
	//	clhr->setLegVelocity(i+1, 6);
	// }

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 1){

		//--------------------------------------------
		// State's transition checking
		//--------------------------------------------

		state = clhr->getLegsPosition();


		//ROS_INFO("Consigna NO alcanzada"); 
		for(int i = 0; i < LEG_NUMBER; i++){
			if((fabs(state[0] < ANG_THR)||(fabs(state[0] - 2*PI) < ANG_THR)) && (!consigna_alcanzada)){
				legs_in_position++;
				ROS_INFO("Pata %d consigna alcanzada -- Consignas alcanzadas: %d", i+1, legs_in_position);
			}
			if(legs_in_position == LEG_NUMBER){
				consigna_alcanzada = true;
				ROS_INFO("Todas las patas en posicion");
			}
		}


		if(legs_in_position == LEG_NUMBER){
				ROS_INFO("Consigna alcanzada. Esperando nueva consigna");

				//ROS_INFO("Actualizar consigna");
				// Volcar consignas anteriores
				for(int i = 0; i < LEG_NUMBER; i++){
					legs_consigna_old[i] = legs_consigna[i];
				}

				//Actualizar consignas patas
				update_movement_parameters(clhr->getArgs());
				legs_consigna = {movement_parameters["leg_1"],
								 movement_parameters["leg_2"],
								 movement_parameters["leg_3"],
								 movement_parameters["leg_4"], 
								 movement_parameters["leg_5"],
								 movement_parameters["leg_6"]};

				//Actualizo solo si hay un valor nuevo
				for(int i = 0; i < LEG_NUMBER; i++){
					if(legs_consigna[i] != legs_consigna_old[i]){
						ROS_INFO("Nueva consigna para la pata %d", i+1);
						clhr->setLegPosition(i+1, legs_consigna[i], stand_up_vel);
						legs_in_position = 0;
						consigna_alcanzada = false;
					}
					clhr->sendCommands();
				}
		}

		loop_rate.sleep();
	}

	return;
}

//Estado 2
/*void state_2 (clhero::Clhero_robot* clhr){

	//------------------------------------------------
	// State's loop rate
	//------------------------------------------------

	ros::Rate loop_rate(STATE_LOOP_RATE);

	ROS_INFO("[FSM_AI]: Entered state 2");

	//------------------------------------------------
	// State's initial statement
	//------------------------------------------------

	//As for example
	// for(int i=0; i<6; i++){
	// 	clhr->setLegPosition(i+1, 90);
	//	clhr->setLegVelocity(i+1, 6);
	// }

	//------------------------------------------------
	// State's core loop
	//------------------------------------------------

	while(clhr->activeState() == 1){

		//--------------------------------------------
		// State's transition checking
		//--------------------------------------------

		//Here the state shall check for transitions
		//in case the conditions for a transition are
		//met, this shall be done by:
		//	clhr->transition(new_state_id);

		clhr->transition(1);

		loop_rate.sleep();
	}

	return;
}*/

//----------------------------------------------------
//    Main function
//----------------------------------------------------

int main (int argc, char** argv){

	//----------------------------------------------------
	//    ROS starting statements
	//----------------------------------------------------

	ros::init(argc, argv, PATTERN_NAME);
	ros::NodeHandle nh;

	//----------------------------------------------------
	//    Gait pattern initialization
	//----------------------------------------------------

	//set the pattern name
	const std::string pattern_name = PATTERN_NAME;

	//Instantiation of the pattern's class
	clhero::Clhero_robot clhr (pattern_name);

	//Register the new gait pattern
	clhero::registerGaitPattern(pattern_name);

	//----------------------------------------------------
	//    State's instantiation
	//----------------------------------------------------

	//attach the states set
	clhr.attachState(1, state_1, STARTING_STATE);
	//clhr.attachState(2, state_2);

	//----------------------------------------------------
	//    Run
	//----------------------------------------------------

	//Lets the gait pattern run
	clhr.run();

	return 0;

}