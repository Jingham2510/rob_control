#include"frontend.hpp"





//GUI controller constructor
frontend_cntrl::frontend_cntrl() {


    //Page flags
    err_page_flag = false;
    landing_page_flag = true;
    ABB_login_page_flag = false;
    ABB_control_page_flag = false;


    //Pointers so that you only have to access and modify once
    page_flags = {&err_page_flag, &landing_page_flag, &ABB_login_page_flag, &ABB_control_page_flag };

    //State flags
    connected = false;



    //Load config
    load_configs();

}



void frontend_cntrl::landing_page() {


    //Create the window
    ImGui::Begin("Robo Connect", &not_close_window);

    //Display the welcome text
    ImGui::Text("Please select a robot brand:");

    //Create the robot brand buttons
    for (int i = 0; i < ENABLED_MODULES.size(); i++) {
        if (ImGui::Button(ENABLED_MODULES[i].c_str())) {
            *page_flags[ENABLED_LANDING_PAGES[i]] = true;
            *page_flags[LANDING] = false;

        }
    }

    ImGui::End();

}



void frontend_cntrl::load_configs()
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
        ABB_ip_presets.push_back({ name, ip, port });
    }
}


//Creates the ip preset buttons
void frontend_cntrl::load_ABB_ip_presets() {

    
    //Create a button for every preset
    for (int i = 0; i < ABB_ip_presets.size(); i++) {

        //If the button is pressed
        if (ImGui::Button(ABB_ip_presets[i][0].c_str())) {
            ip = ABB_ip_presets[i][1];
            port = ABB_ip_presets[i][2];

        }
        //Ensures preset buttons on same line
        ImGui::SameLine();
        
    }

    //Start new lines for next bits
    ImGui::NewLine();


}


//Landing page for the robot controller
// *page_flag is an externally passed flag that the main loop uses to determine whether to render the page or not
void frontend_cntrl::ABB_landing_page() {




    //Create the login window
    ImGui::Begin("ABB Connect", &not_close_window);
  

    //Setup the ABB connect page

    //Display the welcome text
    ImGui::Text("Please enter the IP and Port!");

    //Ask for the ip and port
    ImGui::InputText("IP", &ip);
    ImGui::InputText("Port", &port);


    //load the ip preset buttons
    load_ABB_ip_presets();

    //Connection Button
    if (ImGui::Button("Connect")) {

        //IP is handled automatically

        //Set the page flags
        *page_flags[ABB_LOGIN] = false;
        *page_flags[ABB_MAIN] = true;  

        //When connecting make sure that we are checking the robot model
        robot_model_known = false;
    }

    //Back button
    if (ImGui::Button("Back")) {
        *page_flags[ABB_LOGIN] = false;
        *page_flags[LANDING] = true;
    }


    ImGui::End();
}






//Connects to and controls an ABB robot at a given ip and port
//Pass the robot by reference
void frontend_cntrl::ABB_control_page() {


    ImGui::Begin("ABB Control", &not_close_window);


    if (!connected) {
        //Attempt to connect to the robot
        ABB_rob = ABB_tcp_client(ip.c_str(), std::stoi(port), &connected);

    }

    //If not connected display the error page
    if (!connected) {

        //Setup the error message page
        err_msg.prev_page = ABB_LOGIN;
        err_msg.msg = "Connection error!";

        *page_flags[ABB_MAIN] = false;
        *page_flags[ERR] = true;

    }


    //Create the robot model if not already done
    if (connected and !robot_model_known) {
        //Get the robot model
        robot_name = ABB_rob.get_model();
         
        //Create the mathematical model of the robot
        rob_model = RobotDH(robot_name);

        std::vector<float> model_pos = rob_model.get_pos();

              
        for (int i = 0; i < model_pos.size(); i++) {
            std::cout << model_pos[i] << "\n";
        }
        robot_model_known = true;
    }

    //Ping button (placeholder)
    if (ImGui::Button("PING!")) {
        ABB_rob.ping();
    }

    ImGui::End();



}


//Create an error box that takes the user back to the previous page 
void frontend_cntrl::error_page() {

    ImGui::Begin("ERROR", &not_close_window);

    //Display the error message
    ImGui::Text(err_msg.msg.c_str());


    //Create the back button
    if (ImGui::Button("Back")) {
        *page_flags[ERR] = false;
        *page_flags[err_msg.prev_page] = true;
        
    }



    ImGui::End();


}


//Close any connections that may be running
void frontend_cntrl::close_connections() {

    //Assume that ABB robot is connected if ABB page is open
    if (ABB_control_page_flag) {
        ABB_rob.close_connection();
    }
    return;
}