#pragma once
#include<../abb_tcp/ABB_tcp_client.hpp>

#include <fstream>
#include<iostream>
#include<chrono>
#include<string>
#include<sstream>


#ifndef TEST_MANAGER
#define TEST_MANAGER

//Determines the tests
class test_manager {

public:
	//Create a robot test manager
	test_manager(ABB_tcp_client);
	//Blank constructor 
	test_manager();

	//Ping 100 times and calculate the latencys
	void latency_test();

	//Do one sweeping movement - proof of concept
	void first_pass_test();

	//Tests currently available
	std::vector<std::string> TESTS = { "latency_test", "first_pass test" };

	//Set the output filepath
	void set_data_path(std::string);



private:

	//Polarity controller
	enum pos_neg {
		POSITIVE = 1,
		NEGATIVE = -1
	};

	//The robot that will complete the tests
	ABB_tcp_client robot;

	//The filepath where the data is stored
	std::string data_path;

	//Calculates the error between desired and current pos
	std::vector<float> calc_line_err(std::vector<float>, float, float, float);

};



#endif