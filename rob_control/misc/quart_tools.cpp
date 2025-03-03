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


	std::cout << "{qw: " << qw << " qx: " << qx << " qy: " << qy << " qz: " << qz << "}\n";

	return;
}


//Returns a rotated quartenion where the rotation is specified by a ZYX euler angle
struct quartenion rotate_q(struct quartenion quart, std::vector<float> zyx) {

	//Convert the euler angle to quartenion
	struct quartenion rot_quart = euler_to_q(zyx);

	print_quart(rot_quart);

	//Multiply the current quart by the euler angle
	quart = rotate_q(quart, rot_quart);
	//Return the new quartenion
	return quart;



}


//Rotate a quartenion by another quartenion (via Hamilton Multiplication)
struct quartenion rotate_q(struct quartenion base_quart, struct quartenion rot_quart) {

	struct quartenion new_quart;

	//Multiply the factors together according to: https://www.3dgep.com/understanding-quaternions/#Quaternions
	new_quart.qw = (base_quart.qw*rot_quart.qw) - (base_quart.qx*rot_quart.qx) - (base_quart.qy*rot_quart.qy) - (base_quart.qz*rot_quart.qz);
	new_quart.qx = (base_quart.qw*rot_quart.qx) + (base_quart.qx*rot_quart.qw) + (base_quart.qy*rot_quart.qz) - (base_quart.qz*rot_quart.qy);
	new_quart.qy = (base_quart.qw * rot_quart.qy) + (base_quart.qy * rot_quart.qw) + (base_quart.qz * rot_quart.qx) - (base_quart.qx * rot_quart.qz);
	new_quart.qz = (base_quart.qw * rot_quart.qz) + (base_quart.qz * rot_quart.qw) + (base_quart.qx * rot_quart.qy) - (base_quart.qy * rot_quart.qx);


	return new_quart;
}

//Reutnrs a quartenion in a float of structure {q1, q2, q3, q4}
std::vector<float> quart_to_float(struct quartenion q) {

	return { q.qw, q.qx, q.qy, q.qz };

}

//Converts an angle in degrees to radians
float deg_to_rad(float deg) {

	return deg * (3.14159/ 180);

}