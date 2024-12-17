#include"frontend.hpp"








void frontend::landing_page(std::vector<bool *> page_flags, std::vector<std::string> modules) {


    std::vector<enum frontend::page_flag> flag_list = { ABB_LOGIN };

    //Create the window
    ImGui::Begin("Robo Connect");

    //Display the welcome text
    ImGui::Text("Please select a robot brand:");

    //Create the robot brand buttons
    for (int i = 0; i < modules.size(); i++) {
        if (ImGui::Button(modules[i].c_str())) {
            *page_flags[flag_list[i]] = true;
            *page_flags[LANDING] = false;

        }
    }

    ImGui::End();

}



//Creates the ip preset buttons
void load_ip_presets(std::vector<std::vector<std::string>> ip_presets, std::string* ip, std::string* port) {

    
    //Create a button for every preset
    for (int i = 0; i < ip_presets.size(); i++) {

        //If the button is pressed
        if (ImGui::Button(ip_presets[i][0].c_str())) {
            *ip = ip_presets[i][1];
            *port = ip_presets[i][2];
        }
        //Ensures preset buttons on same line
        ImGui::SameLine();
        
    }

    //Start new lines for next bits
    ImGui::NewLine();


}


//Landing page for the robot controller
// *page_flag is an externally passed flag that the main loop uses to determine whether to render the page or not
void frontend::ABB_landing_page(std::vector<bool*> page_flags, std::vector<std::vector<std::string>> ip_presets) {

    static std::string ip;
    static std::string port;


    //Create the login window
    ImGui::Begin("ABB Connect");
  

    //Setup the ABB connect page

    //Display the welcome text
    ImGui::Text("Please enter the IP and Port!");

    //Ask for the ip and port
    ImGui::InputText("IP", &ip);
    ImGui::InputText("Port", &port);


    //load the ip preset buttons
    load_ip_presets(ip_presets, &ip, &port);

    //Connection Button
    if (ImGui::Button("Connect")) {
        *page_flags[ABB_LOGIN] = false;
    }

    //Back button
    if (ImGui::Button("Back")) {
        *page_flags[ABB_LOGIN] = false;
        *page_flags[LANDING] = true;
    }


    ImGui::End();
}




void frontend::load_configs(std::vector<std::vector<std::string>> *ip_presets)
{
   
    //Open the ip presets file and store the presets
    std::string curr_line;
    std::ifstream file("config/preset_ips.txt");

    std::vector<std::string> curr_preset;
    

    std::string name;
    std::string ip;
    std::string port;

    int comma_pos;
    std::string delimiter = ",";



    //Iterate line by line 
    while (std::getline(file, curr_line)) {


        int i = 0;

        //Find the next comma 
        while ((comma_pos = curr_line.find(delimiter)) != std::string::npos) {

            if (i == 0) {
                name = curr_line.substr(0, comma_pos);
            }

            if (i == 1) {
                ip = curr_line.substr(0, comma_pos);
            }   

            //Erase the bit already stored
            curr_line.erase(0, comma_pos + delimiter.length());

            i = i + 1;
        }

        //format of the config allows us to know that only the port is left
        port = curr_line;

        //Save the preset
        ip_presets->push_back({ name, ip, port });
    }
}


//Connects to and controls an ABB robot at a given ip and port
void frontend::ABB_control_page(std::vector<bool*> page_flags, std::string ip, int port) {

    //Attempt to connect to the robot


    //Create the robot model as well


}

