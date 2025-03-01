#pragma once

#include<vector>

#ifndef QUART_TOOLS
#define QUART_TOOLS

struct quartenion{

	//Magnitude 
	double qw;

	//I
	double qx;
	//J
	double qy;
	//K
	double qz;

};


//Converts a quartenion to a set of ZYX Euler Angles
std::vector<float> q_to_euler(struct quartenion);

//Converts a ZYX format set of euler angles to a quartenion
struct quartenion euler_to_q(std::vector<float>);

//Rotates a quartenion point by a ZYX euler angle
struct quartenion rotate_q(struct quartenion, std::vector<float>);


#endif