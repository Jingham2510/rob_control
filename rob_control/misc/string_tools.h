#pragma once


#include <algorithm>
#include<cctype>
#include<locale>
#include<vector>
#include<sstream>

#ifndef STRING_TOOLS
#define STRING_TOOLS

//Remove the first character of a string
 std::string ltrim(std::string &);

//Remove the last character of a string
 std::string rtrim(std::string &);

//Remove the first and last character of a string
 std::string lrtrim(std::string &);

 //Splits a string based on a delimiter
 std::vector<std::string> split(std::string, std::string);

//Helper function to create strings from vectors of floats
std::string com_vec_to_string(std::vector<float>);

//Helper function - converts xyz strings to floats
std::vector<float> xyz_str_to_float(std::string xyz);


#endif