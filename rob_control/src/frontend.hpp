#pragma once


#include"../abb_tcp/ABB_tcp_client.hpp"
#include"../modelling/Robot_DH.hpp"
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_stdlib.h"
#include <d3d12.h>
#include <dxgi1_4.h>
#include <tchar.h>
#include<string>
#include<iostream>
#include<fstream>
#include<vector>





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

		



private:



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

	//List of IP presets from the config file
	std::vector <std::vector<std::string>> ABB_ip_presets;


	//Pointers so they can modify text inputs
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



	//Load the configs
	void load_ABB_ip_presets();
	void load_configs();





};






#endif
