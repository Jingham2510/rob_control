#include"frontend.hpp"




//Creates the ip preset buttons
void load_ip_presets(std::vector<std::vector<std::string>> ip_presets, std::string* ip, std::string* port) {

    //Create a button for every preset
    for (int i = 0; i < ip_presets.size(); i++) {

        //If the button is pressed
        if (ImGui::Button(ip_presets[i][0].c_str())) {
            *ip = ip_presets[i][1];
            *port = ip_presets[i][2];
        }
    }


}


//Landing page for the robot controller
// *page_flag is an externally passed flag that the main loop uses to determine whether to render the page or not
void frontend::setup_landing_page(bool *page_flag, std::vector<std::vector<std::string>> ip_presets) {

    static std::string ip;
    static std::string port;


    //Create the login window
    ImGui::Begin("ABB Connect");

    //WILL REQUIRE FORMATTING LATER - FUNCIONALITY FIRST


    //Setup the landing page

    //Display the welcome text
    ImGui::Text("Please enter the IP and Port!");

    //Ask for the ip and port
    ImGui::InputText("IP", &ip);
    ImGui::InputText("Port", &port);


    //load the ip preset buttons
    load_ip_presets(ip_presets, &ip, &port);

    //Connection Button
    if (ImGui::Button("Connect")) {
        std::cout << ip << "\n";
        *page_flag = false;
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

        std::cout << curr_line << "\n";

        int i = 0;

        //Find the next comma
        while ((comma_pos = curr_line.find(delimiter)) != std::string::npos) {

            if (i == 0) {
                //Extract the info - keep unwrapped for now
                name = curr_line.substr(0, comma_pos);
            }

            if (i == 1) {
                ip = curr_line.substr(0, comma_pos);
            }

    

            //Erase the bit already stored
            curr_line.erase(0, comma_pos + delimiter.length());

            i = i + 1;
        }

        port = curr_line;

        //Save the preset
        ip_presets->push_back({ name, ip, port });
    }






}

