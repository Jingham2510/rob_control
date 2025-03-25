#include "string_tools.h"



//String trimming sourced from: https://stackoverflow.com/questions/216823/how-to-trim-a-stdstring

//Remove the first character of a string - inplace
 std::string ltrim(std::string &s) {
    s.erase(0,1);
    return s;
}

//TRim from the end of a string (in place)
 std::string rtrim(std::string &s) {
     /*
     s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
         return !std::isspace(ch);
         }).base(), s.end());
    */
        
     s.erase(s.size() - 1, 1);


     return s;
}


std::string lrtrim(std::string &s) {
	s = ltrim(s);

	s = rtrim(s);

    return s;
}

std::vector<std::string> split(std::string s, std::string delim) {

    size_t pos = 0;

    std::vector<std::string> tokens;
    std::string token;
    while ((pos = s.find(delim)) != std::string::npos) {
        token = s.substr(0, pos);
        tokens.push_back(token);
        s.erase(0, pos + delim.length());
    }

    tokens.push_back(s);

    return tokens;
}



 //Takes in a vector of floats then returns it as a string - comma seperated
 std::string com_vec_to_string(std::vector<float> data) {

     std::stringstream stream;

     //Goes through the whole array and adds it
     for (size_t i = 0; i < data.size(); i++) {

         stream << data[i];

         if (i != data.size() - 1) {
             stream << ",";
         }

     }

     return stream.str();

 }

 //Turns the RAPID xyz string to a vector
 std::vector<float> xyz_str_to_float(std::string xyz) {


     //std::cout << xyz << "\n";

     std::vector<float> xyz_vec;

     //Split the string (delimited by ",") - slicing off the front and end square brackets
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