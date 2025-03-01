#include "quart_tools.h"
#include<numbers>

//NOTE: ZYX - YAW PITCH ROLL


//Converts a quartenion to a set of ZYX Euler Angles
std::vector<float> q_to_euler(struct quartenion quart) {

	//Define the outputs
	float pitch;
	float yaw;
	float roll;


	//Check if the pitch is equal to +/-90 deg (gimbal lock)
	pitch = asin(2 * (quart.qw * quart.qy - quart.qx * quart.qz));


	if (pitch == (std::numbers::pi)/2) {
		//Pitch is positively gimbal locked
		roll = 0;
		yaw = -2 * atan2(quart.qx, quart.qw);
		
	}
	else if(pitch == -(std::numbers::pi)/2) {
		//Pitch is negatively gimbal locked
		roll = 0;
		yaw = 2 * atan2(quart.qx, quart.qw);
	}

	else {
		//No gimbal lock
		roll = atan2((2*(quart.qw*quart.qx + quart.qy*quart.qz)),(pow(quart.qw, 2) - pow(quart.qx, 2) - pow(quart.qy, 2) + pow(quart.qz, 2)));

		yaw = atan2((2 * (quart.qw * quart.qz + quart.qx * quart.qy)), (pow(quart.qw, 2) + pow(quart.qx, 2) - pow(quart.qy, 2) - pow(quart.qz, 2)));
	}




	return { yaw, pitch, roll };

}

