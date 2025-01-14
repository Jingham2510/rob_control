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





//Wrapper function which calls all the config loaders
void frontend_cntrl::load_configs()
{
    //load ip configs
    load_ip_configs();

}


void frontend_cntrl::load_ip_configs() {

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

        //Create the robot test manager
        //Load test manager
        test_mgr = test_manager(&ABB_rob);

        //Get the robot model
        robot_name = ABB_rob.get_model();

        //Create the mathematical model of the robot
        rob_model = RobotDH(robot_name);

        //If the model is valid 
        if (rob_model.is_valid()) {
            //Update the models joint angles with the real robots joints to 
            rob_model.update_joints(ABB_rob.get_last_reported_jnt_angs());
            std::cout << "Joints calced";
         }
        //Still mark as known - it is known just as invalid
        robot_model_known = true;
    }
        /*
            TODO:
            -BUTTONS TO CREATE MODULES
            -MODEL VS REPORTED ERROR PLOT
            -Button Control
            -Angle Chooser
        */

        //Create the title for the page
        std::stringstream title_stream;
        title_stream << "Robot Model: " << robot_name;
        ImGui::Text(title_stream.str().c_str());



        //Setup the headers for each section

        //Robot info
        if (ImGui::CollapsingHeader("Unit Info")) {
            load_robot_info();
        }

        //Model Robot Info
        if (ImGui::CollapsingHeader("Model Info")) {

            //TODO : MOVE TO SEPERATE FUNCTION

            if (rob_model.is_valid()) {
                
                std::vector<std::string> cart_cords = { "X: ", " Y: ", " Z: " };
                
                std::vector<float> pos = rob_model.get_pos();

                std::stringstream disp_text;

                //Display the models position - for verification
                ImGui::Text("Model Pos");

                for (int i = 0; i < pos.size(); i++) {

                    disp_text << cart_cords[i] << pos[i];

                    ImGui::Text(disp_text.str().c_str());

                    ImGui::SameLine();

                    disp_text.str("");
                }

                ImGui::NewLine();




            }


            else {
                ImGui::Text("Robot Model not known!");
            }




        }

        //Manual Control Section
        if (ImGui::CollapsingHeader("Manual Control")) {
            //Create the manual control section
            load_man_control();

        }

        //Load the test section
        if (ImGui::CollapsingHeader("Tests")) {
            load_test_section();
        }


        //Ping test - PLACEHOLDER
        if (ImGui::CollapsingHeader("PING TEST")) {

            if (!test_mgr.test_running()) {
                if (ImGui::Button("PING!")) {
                    ABB_rob.ping();
                }
            }
            else {
                ImGui::Text("Test running...");
            }
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

//Load the robots info into the page
void frontend_cntrl::load_robot_info() {

    //If the robot is an ABB one
    if (ABB_control_page_flag) {

        //If a test is running
        if (test_mgr.test_running()) {
            ImGui::Text("Test running...");
        }
        else {
            ImGui::Text("No test running...");
        }
        
        ImGui::Separator();

        //Position IDs
        std::vector<std::string> ids = { "X: ", "Y: ", "Z: ", "Rx: ", "Ry: ", "Rz: "};
        //Generate the text to display
        std::stringstream disp_text;

        //Get the last reported position
        std::vector<float> curr_pos = ABB_rob.get_last_reported_pos();

        ImGui::Text("Cartesian Position");
 
        //Display the currently reported position of the robot
        for (int i = 0; i < ids.size()/2; i++) {            

            disp_text << ids[i] << curr_pos[i] << " ";


            ImGui::Text(disp_text.str().c_str());           

            ImGui::SameLine();

            //Clear the string stream
            disp_text.str("");
        }


        ImGui::NewLine();
        
        ImGui::Separator();

        //Display the orientation as well
        ImGui::Text("Euler Angles");

        
        //Get the last reported orientation
        std::vector<float> curr_ori = ABB_rob.get_last_reported_ori();

        

        for (int i = 0; i < ids.size()/2; i++) {

            disp_text << ids[i] << curr_ori[i] << " ";

            ImGui::Text(disp_text.str().c_str());

            ImGui::SameLine();

            //Clear the string stream
            disp_text.str("");
        }
        
        ImGui::NewLine();

        ImGui::Separator();
       
        //Display the current force
        ImGui::Text("Force");

        std::vector<float> curr_force = ABB_rob.get_last_reported_force();
        
        for (int i = 0; i < ids.size(); i++) {
            
            disp_text << ids[i] << curr_force[i] << " ";

            ImGui::Text(disp_text.str().c_str());

            ImGui::SameLine();

            //Clear the string stream
            disp_text.str("");

        }


        ImGui::NewLine();  

        ImGui::Separator();
        //Display the joint angles
        ImGui::Text("Joint Angles");
        
        std::vector<float> jnt_angs = ABB_rob.get_last_reported_jnt_angs();

        std::stringstream angs_text;
        angs_text << "J1: " << jnt_angs[0] << " | J2: " << jnt_angs[1] << " | J3: " << jnt_angs[2] << " | J4: " << jnt_angs[3] << " | J5:  " << jnt_angs[4] << " | J6: " << jnt_angs[5];
        ImGui::Text(angs_text.str().c_str());

    }

    return;
}


//Load the manual control buttons
void frontend_cntrl::load_man_control() {

    //Place button which moves the tool up
    if (ImGui::Button("Up")) {
        ABB_rob.move_tool({ 0, 0, -10 });
    }
    

    //Place button which moves the tool up
    if (ImGui::Button("Down")) {
        ABB_rob.move_tool({ 0, 0, 10 });
    }

    ImGui::Text("MORE TODO!");

    return;
}

//Load the section for running and monitoring tests
void frontend_cntrl::load_test_section() {

    //Combo/dropdown variables
    static const char* current_item = NULL;

  

    //Create dropdown with test selection
    if (ImGui::BeginCombo("##Tests", current_item)) {


        //Add the selectable tests
        for (int i = 0; i < test_mgr.TESTS.size(); i++) {
            
            //check if item is selected - this works?
            bool is_selected = ("test" == test_mgr.TESTS[i]);
                 

            if (ImGui::Selectable(test_mgr.TESTS[i].c_str(), is_selected)) {

                //Set the current item
                current_item = test_mgr.TESTS[i].c_str();

                data_fp = "C:/Users/User/Documents/Results/" + test_mgr.TESTS[i] + "/raw/" ;
                
            }

            if (is_selected) {
                ImGui::SetItemDefaultFocus();
                             
            }



        }

        ImGui::EndCombo();
    }


    //Create the filepath box
    ImGui::InputText("Filepath", &data_fp);
 

    //Only allow test button if test running
    if (!test_mgr.test_running()) {

        //Run test button
        if (ImGui::Button("Run Test")) {
            //Setup the test manager
            test_mgr.set_data_path(data_fp);
            //Select the test to run
            test_mgr.test_selector(current_item);
        }
    }

    else{
        ImGui::Text("TEST RUNNING"); 
     }

    return;
}