#pragma once


#include"../abb_tcp/ABB_tcp_client.hpp"
#include"../modelling/Robot_DH.hpp"
#include"../abb_tcp/tests/test_manager.hpp"
#include "imgui.h"
#include "implot.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_stdlib.h"
#include <d3d12.h>
#include <dxgi1_4.h>
#include <tchar.h>
#include<string>
#include<sstream>
#include<iostream>
#include<fstream>
#include<vector>
#include"../misc/string_tools.h"

#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>


#ifndef FRONTEND
#define FRONTEND







class frontend_cntrl {



public:


	//Allows ImGui to know which pages to show
	bool err_page_flag;
	bool landing_page_flag;
	bool ABB_login_page_flag;
	bool ABB_control_page_flag;

	//Close window flag
	bool not_close_window = true;


	//Test manager
	test_manager test_mgr;


	//Constructor
	frontend_cntrl();


	//Opening page which shows vailable modules
	void landing_page();

	//ABB ip connection page
	void ABB_landing_page();
	//ABB robot control page
	void ABB_control_page();



	//Displays in case of an error
	void error_page();

	//Close any connections that may be running
	void close_connections();
		



private:


	//Debug testing flag
	bool TEST_FLAG = 0;


	//Flag vector accessor
	const enum page_flag {
		ERR = 0,
		LANDING = 1,
		ABB_LOGIN = 2,
		ABB_MAIN = 3
	};


	//Modules that are enabled for use - and the associated landing page flag
	const std::vector<std::string> ENABLED_MODULES = { "ABB" };
	const std::vector<enum page_flag> ENABLED_LANDING_PAGES = { ABB_LOGIN };

	//defines what is displayed on the error page
	struct err_msg {
		enum page_flag prev_page;
		std::string msg;


	}err_msg;

	//pointers are private as only the gui should modify them
	std::vector<bool*> page_flags;

	//Custom trajecotry close window flag
	bool not_close_cust_traj_wind = false;
	//Custom Test Generator Window
	void cust_traj_generator();


	//Camera checker close window flag
	bool not_close_cam_page = false;
	//Camera checker window
	void cam_window();

	//List of IP presets from the config file
	std::vector <std::vector<std::string>> ABB_ip_presets;

	//List of valid tests loaded from the test config file
	std::vector <std::string> test_titles;

	//IP to connect to
	std::string ip;
	//Port to connect to
	std::string port;

	//State flags
	//Connected to a device
	bool connected;

	//Known robot model
	bool robot_model_known;
	std::string robot_name;

	//Create a blank model
	RobotDH rob_model;

	//Robot placeholders
	ABB_tcp_client ABB_rob;
	//Test output filepath
	std::string data_fp;


	//Custom test trajectory points
	std::vector<ImVec2> cust_pnts;
	bool redo_valid = false;
	ImVec2 redo_buff;
	
	//Custom trajectory settings
	std::string cust_name;
	int rob_speed = 25;
	int rob_height = 85;

	//Load the configs into the gui
	void load_ABB_ip_presets();

	//Load the configs from the config text files
	void load_configs();
	void load_ip_configs();




	//Control page dropdown sections -----

	//Load the robot info
	void load_robot_info();

	//Load the manual control buttons
	void load_man_control();

	//Test section - for running experiments
	void load_test_section();






};


#endif
