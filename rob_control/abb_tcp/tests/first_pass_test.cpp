/*
First pass test
Routine:
-Run from a given coordinate to another coordinate (in a straight line)
-Log curr position and force on robot 
-Use control method to ensure height stays relatively the same
-Take snapshot of depth camera measurement
-Save all data to text file (format: count, time,{pos,rot},{forces})


*/


#include"../ABB_tcp_client.hpp"
#include<chrono>
#include<string>
#include<sstream>
#include<vector>
#include<fstream>
#include<iostream>


//#include"../../misc/date.h"
//using namespace date;



enum pos_neg {
	POSITIVE = 1,
	NEGATIVE = -1
};



//Turns the RAPID xyz string to a vector
std::vector<float> xyz_str_to_float(std::string xyz) {

	std::vector<float> xyz_vec;

	//Split the string (delimited by ",") - sliving off the front and end square brackets
	std::stringstream ss(xyz.substr(1, xyz.length()));
	std::string item;
	std::vector<std::string> elems;
	while (std::getline(ss, item, ',')) {
		elems.push_back(std::move(item));
	}

	//Turn them to vectors
	for (int i = 0; i < elems.size(); i++) {
		xyz_vec.push_back(std::stof(elems[i]));
	}


	return xyz_vec;
}

//Calculates the robots position error from the desired error
std::vector<float> calc_line_err(std::vector<float> curr_xyz, float desired_x, float desired_y, float desired_z) {


	std::cout << desired_y << "\n";



	//Create the vector float from the string
	std::vector<float> curr_xyz_vec = curr_xyz;



	//Create the vector from the differences (invert to ensure the tool moves the correct direction)
	std::vector<float> xyz_diff = {-(desired_x - curr_xyz_vec[0]), (desired_y - curr_xyz_vec[1]), -(desired_z - curr_xyz_vec[2])};


	for(int i = 0; i < xyz_diff.size(); i++){
	
		std::cout<< "CURR: " << curr_xyz_vec[i] << " DIFF: " << xyz_diff[i] << "\n";
	}

	return xyz_diff;


}

//The main test loop
int first_pass() {

	std::vector<std::chrono::system_clock::time_point> time_data;
	//Stores both the force and posiiton data as a string
	//This is because we don't need to do any data analysis here
	std::vector<std::string> force_pos_data;
	std::string curr_force_pos;

	//TO BE DETERMINED - filler for now! - be very careful when doing it on the real one...
	float DES_X = 2200;
	float DES_Z = 190;
	std::vector<float> START_POS = {DES_X, -788, DES_Z};
	std::vector<float> END_POS = {DES_X, 315, DES_Z };
	//Steps based on distance and resolution of test
	int STEP_RES = 1;
	int dist_to_move = START_POS[1] - END_POS[1];
	enum pos_neg polarity = POSITIVE;
	
	//Ensure that the direction is correct
	if (START_POS[1] > END_POS[1]) {
		dist_to_move = -dist_to_move;
		polarity = NEGATIVE;
	}
	
	int steps = abs(dist_to_move * STEP_RES);
	std::vector<float> move_vector;

	//Connect to the ABB robot
	//ABB_tcp_client client = ABB_tcp_client("192.168.125.1", 8888);

	bool connected = false;

	ABB_tcp_client client = ABB_tcp_client("127.0.0.1", 8888, &connected);

	//Move the robot to the starting position - checked manually for now
	client.set_joints({-20.01, 58.59, 46.97, 4.81, 43.57, -32.28});

	//Move the robot and log the data
	for (int i = 0; i <= steps; i++) {

		//Calculate the error from the desired pos
		move_vector = calc_line_err(client.req_xyz(), DES_X, START_POS[1] + (polarity * i * STEP_RES), DES_Z);		

		//move_vector = {0, 2, 0};

		//Move the robot - ensuring that the tool attempts to stay in a straight line
		curr_force_pos = client.move_tool(move_vector);

		//Place the data in the arrays
		time_data.push_back(std::chrono::system_clock::now());

		force_pos_data.push_back(curr_force_pos);

		std::cout << curr_force_pos << "\n";
	}


	//Close the socket
	client.close_connection();

	//Create the filename
	std::stringstream filename;
	filename << "C:/Users/User/Documents/Results/first_pass_tests/raw/" << "placeholder" << "_first_pass.txt";

	//Save the data to a logfile
	std::ofstream data_file(filename.str());
	for (int i = 0; i < time_data.size(); i++) {
		//data_file << i << "," << time_data[i] << "," << force_pos_data[i] << "\n";
		data_file << i << "," << "PLACEHOLDER" << "," << force_pos_data[i] << "\n";
	}
	data_file.close();

	return 1;
}



