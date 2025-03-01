#include "quart_tools.h"
#include<numbers>
#include<string>
#include<iostream>

//NOTE: ZYX - YAW PITCH ROLL

//Implementations/Calculations taken from
//https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

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


//Converts a ZYX format set of euler angles to a quartenion
struct quartenion euler_to_q(std::vector<float> zyx) {
	struct quartenion quart;

	quart.qw = (cos(zyx[2]/2) * cos(zyx[1]/2) * cos(zyx[0]/2)) + (sin(zyx[2]/2) * sin(zyx[1]/2) * sin(zyx[0]/2));

	quart.qx = (sin(zyx[2]/2) * cos(zyx[1]/2) * cos(zyx[0]/2)) - (cos(zyx[2]/2) * sin(zyx[1]/2) * sin(zyx[0]/2));

	quart.qy = (cos(zyx[2]/2) * sin(zyx[1]/2) * cos(zyx[0]/2)) + (sin(zyx[2]/2) * cos(zyx[1]/2) * sin(zyx[0]/2));

	quart.qz = (cos(zyx[2]/2) * cos(zyx[1]/2) * sin(zyx[0]/2)) - (sin(zyx[2]/2) * sin(zyx[1]/2) * cos(zyx[0]/2));


	return quart;

}


//Print the quartenions
void print_quart(struct quartenion quart){

	//Convert the quartenion factors to strings
	std::string qw = std::to_string(quart.qw);
	std::string qx = std::to_string(quart.qx);
	std::string qy = std::to_string(quart.qy);
	std::string qz = std::to_string(quart.qz);


	std::cout << "{qw: " << qw << " qx: " << qx << " qy: " << qy << " qz: " << qz << "}";

	return;
}