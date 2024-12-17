#pragma once

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



namespace frontend {

	//Flag vector accessor
	enum page_flag {
		LANDING = 0,
		ABB_LOGIN = 1
	};



	void landing_page(std::vector<bool *>, std::vector<std::string>);


	void ABB_landing_page(std::vector<bool *>, std::vector<std::vector<std::string>>);

	void load_configs(std::vector<std::vector<std::string>>*);

}



#endif
