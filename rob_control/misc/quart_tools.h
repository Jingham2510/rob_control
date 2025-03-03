#pragma once

#include<vector>

#ifndef QUART_TOOLS
#define QUART_TOOLS

struct quartenion{

	//Magnitude 
	float qw;

	//I
	float qx;
	//J
	float qy;
	//K
	float qz;

};


//Converts a quartenion to a set of ZYX Euler Angles
std::vector<float> q_to_euler(struct quartenion);

//Converts a ZYX format set of euler angles to a quartenion
struct quartenion euler_to_q(std::vector<float>);

//Prints out a quartenion
void print_quart(struct quartenion);

//Multiply two quartenions together (rotate the first by the second)
struct quartenion rotate_q(struct quartenion, struct quartenion);


//Returns a rotated quartenion where the rotation is specified by a ZYX euler angle
struct quartenion rotate_q(struct quartenion, std::vector<float>);

//Reutnrs a quartenion in a float of structure {q1, q2, q3, q4}
std::vector<float> quart_to_float(struct quartenion);

//Converts an angle in degrees to radians
float deg_to_rad(float);

#endif