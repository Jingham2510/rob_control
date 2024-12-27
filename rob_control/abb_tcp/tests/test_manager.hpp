#pragma once
#include<../abb_tcp/ABB_tcp_client.hpp>

#include <fstream>
#include<iostream>
#include<chrono>
#include<string>
#include<sstream>
#include<thread>
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_stdlib.h"
#include <d3d12.h>
#include <dxgi1_4.h>



#ifndef TEST_MANAGER
#define TEST_MANAGER

//Determines the tests
class test_manager {

public:
	//Create a robot test manager
	test_manager(ABB_tcp_client);
	//Blank constructor 
	test_manager();


	//Tests currently available
	bool LATENCY_TEST_FLAG = false;
	bool FIRST_PASS_FLAG = false;
	

	std::vector<std::string> TESTS = { "latency_test", "first_pass_test" };

	//Set the output filepath
	void set_data_path(std::string);

	//Get the output filepaht
	std::string get_data_path();

	//Report whether a test is running
	bool test_running();

	//Selects a test to run
	void test_selector(std::string);


	//Ping 100 times and calculate the latencys
	void latency_test();

	//Do one sweeping movement - proof of concept
	void first_pass_test();


private:

	//Polarity controller
	enum pos_neg {
		POSITIVE = 1,
		NEGATIVE = -1
	};


	//Test state controllers
	bool test_complete;
	bool file_saved;
	bool close;
	int loop_counter;

	//Generic Test storages
	std::vector<float> storage_1;
	std::vector<float> storage_2;
	//String storages
	std::vector<std::string> string_storage;
	std::vector<std::chrono::system_clock::time_point> time_data;


	//Generic test flags
	bool TEST_FLAG_1;
	bool TEST_FLAG_2;
	bool TEST_FLAG_3;


	//Flag to indicate if a test is running
	bool TEST_RUNNING_FLAG = false;

	//The robot that will complete the tests
	ABB_tcp_client robot;

	//The filepath where the data is stored
	std::string data_path;

	//Calculates the error between desired and current pos
	std::vector<float> calc_line_err(std::vector<float>, float, float, float);




};



#endif