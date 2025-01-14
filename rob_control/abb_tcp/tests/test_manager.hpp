#pragma once
#include<../abb_tcp/ABB_tcp_client.hpp>

#include <fstream>
#include<iostream>
#include<chrono>
#include<string>
#include<sstream>
#include<thread>
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_stdlib.h"
#include <d3d12.h>
#include <dxgi1_4.h>
#include<format>



#ifndef TEST_MANAGER
#define TEST_MANAGER

//Determines the tests
class test_manager {

public:
	//Create a robot test manager
	test_manager(ABB_tcp_client*);
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


	// utility structure for realtime plot - nicked from IMplot demo
	struct RollingBuffer {
		float Span;
		ImVector<ImVec2> Data;
		RollingBuffer() {
			Span = 10.0f;
			Data.reserve(2000);
		}
		void AddPoint(float x, float y) {
			float xmod = fmodf(x, Span);
			if (!Data.empty() && xmod < Data.back().x)
				Data.shrink(0);
			Data.push_back(ImVec2(xmod, y));
		}
	};

	//Test state controllers
	bool test_complete;
	bool file_saved;
	bool close;
	int loop_counter;

	int spare_counter;

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
	ABB_tcp_client *robot;

	//The filepath where the data is stored
	std::string data_path;

	//Provides live latency plot for the latency test
	void latency_plotting(float);

	//Provides plotting for general force-displacement test
	//Plots current xyz pos, current xyz error, current 6-axis force
	// TODO: Added orientation bits
	void force_displacement_plotting(std::vector<float>);

	//Calculates the error between desired and current pos
	std::vector<float> calc_line_err(std::vector<float>, float, float, float);



	//Sequential vertex control variables
	//The current vector that is stepped along
	std::vector<float> seq_curr_mov_vec;
	//The direction  of the movement on each axis
	std::vector<int> seq_curr_xyz_dir = {1, 1, 1};


	//Moves to every coordinate given in the vector
	//at a rate/speed determined by the step size
	void sequential_vertex_move(std::vector<std::vector<float>>, float);



};



#endif