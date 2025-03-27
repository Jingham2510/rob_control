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
        if (ImGui::CollapsingHeader("TEST FUNC")) {
          

            if (ImGui::Button("Test")) {

                not_close_cust_traj_wind = true;
            }            
           
        }

        //Open the custom trajectory painter window
        if (not_close_cust_traj_wind == 1) {
            cust_traj_generator();
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

    if (ImGui::Button("Y++")) {
        ABB_rob.move_tool({ 0, 10, 0 });
    }

    if (ImGui::Button("X++")) {
        ABB_rob.move_tool({ 10, 0, 0 });
    }

    ImGui::Text("MORE TODO!");

    return;
}

//Load the section for running and monitoring tests
void frontend_cntrl::load_test_section() {


    //Test setup
    ImGui::Text("Test Setup");

    //Weight calibration button - to be implemented
    ImGui::Text("Weight cali to be implemented");


    //Setup orientation button (straight down)
    if (ImGui::Button("Point TCP Down")) {
        ABB_rob.set_ori({ 0.02443,-0.70660,0.70658,0.02939 });
    }    


    ImGui::Separator();


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



//Opens the window where the user generates the custom tests
void frontend_cntrl::cust_traj_generator() {

    //Create the window at a given size
    ImGui::SetNextWindowSize(ImVec2(950, 950));
    ImGui::Begin("Custom Trajectory Painter", &not_close_cust_traj_wind);

    ImGui::Text("Soilbox Trajectory");


    //Point drawing undo/redo logic ----------------

    ImGui::BeginDisabled(!(cust_pnts.size() > 0));

    //Remove the last point
    if (ImGui::Button("Undo")) {
        std::cout << "UNDO\n";

        if (cust_pnts.size() > 0) {
            redo_buff = cust_pnts[cust_pnts.size() - 1];
            cust_pnts.pop_back();
            redo_valid = true;
        }
    }
    ImGui::EndDisabled();

    ImGui::SameLine();

    ImGui::BeginDisabled(!redo_valid);
  
    //Readd an undoed point point
    if (ImGui::Button("Redo")) {
        std::cout << "REDO\n";

        if (redo_valid) {
            cust_pnts.push_back(redo_buff);
            redo_valid = false;
        }       
    }   
    ImGui::EndDisabled();
  


    ImGui::NewLine();

    //Draw the canvas region!--------------------------
    float CANVAS_SIZE = 500;
    //Draw the canvas at the location of the screen cursor
    ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
    ImVec2 canvas_sz = ImVec2(CANVAS_SIZE, CANVAS_SIZE);
    ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);


    // Draw border and background color
    ImGuiIO& io = ImGui::GetIO();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
    draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

    //Draw the boundaries of the soilbox
    std::vector<double> sandbox_boundaries = { 0.1 * CANVAS_SIZE, 0.1 * CANVAS_SIZE, 0.9 * CANVAS_SIZE , 0.9 * CANVAS_SIZE };

    draw_list->AddLine(ImVec2(canvas_p0.x + sandbox_boundaries[0], canvas_p0.y + sandbox_boundaries[1]),
        ImVec2(canvas_p0.x + sandbox_boundaries[0], canvas_p0.y + sandbox_boundaries[3]), IM_COL32(0, 255, 0, 255));

    draw_list->AddLine(ImVec2(canvas_p0.x + sandbox_boundaries[0], canvas_p0.y + sandbox_boundaries[1]),
        ImVec2(canvas_p0.x + sandbox_boundaries[2], canvas_p0.y + sandbox_boundaries[1]), IM_COL32(0, 255, 0, 255));

    draw_list->AddLine(ImVec2(canvas_p0.x + sandbox_boundaries[2], canvas_p0.y + sandbox_boundaries[3]),
        ImVec2(canvas_p0.x + sandbox_boundaries[2], canvas_p0.y + sandbox_boundaries[1]), IM_COL32(0, 255, 0, 255));

    draw_list->AddLine(ImVec2(canvas_p0.x + sandbox_boundaries[2], canvas_p0.y + sandbox_boundaries[3]),
        ImVec2(canvas_p0.x + sandbox_boundaries[0], canvas_p0.y + sandbox_boundaries[3]), IM_COL32(0, 255, 0, 255));


    //Draw the approximate working boundaries of the robot

    //Scaled - 2000mm for no of canvas pixels that take up the valid sandbox space

    float POINT_SCALING = 2000 / (CANVAS_SIZE*0.8);

    //MAgic numbers are measurements from box 
    ImVec2 bound_p0 = ImVec2((canvas_p0.x + sandbox_boundaries[0]), (canvas_p0.y + sandbox_boundaries[3]) - (1352 / POINT_SCALING));
    ImVec2 bound_p1 = ImVec2((canvas_p0.x + sandbox_boundaries[0]) + (382 / POINT_SCALING), (canvas_p0.y + sandbox_boundaries[3]) - (1487 / POINT_SCALING));
    ImVec2 bound_p2 = ImVec2((canvas_p0.x + sandbox_boundaries[0]) + (1395 / POINT_SCALING), (canvas_p0.y + sandbox_boundaries[3]) - (1487 / POINT_SCALING));
    ImVec2 bound_p3 = ImVec2((canvas_p0.x + sandbox_boundaries[2]), (canvas_p0.y + sandbox_boundaries[3]) - (1352 / POINT_SCALING));

    draw_list->AddLine(bound_p0, bound_p1, IM_COL32(0, 0, 255, 255));
    draw_list->AddLine(bound_p1, bound_p2, IM_COL32(0, 0, 255, 255));
    draw_list->AddLine(bound_p2, bound_p3, IM_COL32(0, 0, 255, 255));



    //Check the moues coordinates
        //If on the canvas

    const ImVec2 mouse_pos_in_canvas(io.MousePos.x - canvas_p0.x, io.MousePos.y - canvas_p0.y);
    bool valid_pos = false;

    if (mouse_pos_in_canvas.x > 0 and io.MousePos.x < canvas_p1.x and mouse_pos_in_canvas.y > 0 and mouse_pos_in_canvas.y < canvas_p1.y) {
 
        //If outside the soilbox position
        if (mouse_pos_in_canvas.x < sandbox_boundaries[0] or mouse_pos_in_canvas.x > sandbox_boundaries[2] or mouse_pos_in_canvas.y < sandbox_boundaries[1] or mouse_pos_in_canvas.y > sandbox_boundaries[3]) {
            //Set flag
            valid_pos = false;
            //Change mouse icon
            ImGui::SetMouseCursor(ImGuiMouseCursor_NotAllowed);
        }
        else {
            valid_pos = true;
            ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
        }
    }
    else {
        valid_pos = false;
        ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
     }


    //Point Drawing Logic-------------------------------------

    ImColor pnt_col = IM_COL32(255, 0, 0, 255);
    float pnt_rad = 5;

    

    //Check if user is clicking in a valid spot
    if (valid_pos and ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {

        //Save the new point
        cust_pnts.push_back(mouse_pos_in_canvas);

    }

    //Draw the points
    for (int i = 0; i < cust_pnts.size(); i++) {

        ImVec2 draw_pos = ImVec2(cust_pnts[i].x + canvas_p0.x, cust_pnts[i].y + canvas_p0.y);


        draw_list->AddCircle(draw_pos, pnt_rad, pnt_col);

        //If the points list is greater than one add the line to connect them
        if (i > 0) {

            ImVec2 prev_pos = ImVec2(cust_pnts[i - 1].x + canvas_p0.x, cust_pnts[i - 1].y + canvas_p0.y);

            draw_list->AddLine(prev_pos, draw_pos, pnt_col);
        }


    }

 
    //Settings Logic-------------------------------------------

   


    ImGui::TextColored(ImVec4(0, 255, 0, 255), "Green lines are the soilbox boundaries");
    //Move the cursor across the page
    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));
    

    //Draw the Settings Section
    ImGui::Text("Settings");

    ImGui::TextColored(ImVec4(0, 255, 0, 255), "Blue lines are approximate robot working boundaries");

    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));
    ImGui::Separator();

    //Test Name
    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));
    ImGui::SetNextItemWidth(200);
    ImGui::InputText("Test Name", &cust_name);

    //Robot Speed
    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));
    ImGui::SetNextItemWidth(200);
    ImGui::SliderInt("Robot Speed", &rob_speed, 0, 1000);
    //Robot Height
    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));
    ImGui::SetNextItemWidth(200);
    ImGui::SliderInt("Robot Height", &rob_height, 70, 300);


    //Save/Load Trajectory logic---------------
    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));
    
    //Ensure there are points to be saved & that a file name exists
    ImGui::BeginDisabled((cust_name.empty()) or (cust_pnts.empty()));
    if (ImGui::Button("Save Trajectory")) {
        std::ofstream data_file("cust_trajcs/" + cust_name + ".traj");
        //Save every point seperated by a comma
        for (int i = 0; i < cust_pnts.size(); i++) {
            data_file << "(" << std::to_string(cust_pnts[i].x) << " " << std::to_string(cust_pnts[i].y) << "),";
        }

    }
    ImGui::EndDisabled();


    ImGui::SameLine();
    if (ImGui::Button("Load Trajectory")) {

     
        //Open the file specified by the trajectory name
        std::ifstream data_file("cust_trajcs/" + cust_name + ".traj");

        //Check the file opened okay
        if (data_file.good()) {
            //Clear the current points
            cust_pnts.clear();     


            std::stringstream contents;
            contents << data_file.rdbuf();
            //Extract all the file info
            std::string file_contents = contents.str();

            //Split the string into seperate coordinate points
            std::vector<std::string> str_pnts = split(file_contents, ",");
            //For each string point - convert it to an ImVec and add to the draw list
            //-1 to ignore the final new line
            for (int i = 0; i < str_pnts.size() - 1; i++) {
                //Access the point
                std::string curr_pnt = str_pnts[i];
       
                //Remove the front and end brackets
                curr_pnt = lrtrim(curr_pnt);

                //Split the string into two floats delimited by the space
                std::vector<std::string> coords = split(curr_pnt, " ");

                //Create the ImVec2 point from the floats
                ImVec2 curr_vec = ImVec2(std::stof(coords[0]), std::stof(coords[1]));
            
                //Add the points to the point lists
                cust_pnts.push_back(curr_vec);
            

            }
        }

        else {

            //Setup the error message page
            err_msg.prev_page = ABB_MAIN;
            err_msg.msg = "No such trajectory!";

            *page_flags[ABB_MAIN] = false;
            *page_flags[ERR] = true;


        }

    }


    ImGui::NewLine();




    //START LOGIC -----------------------------
  
    //Create the start button
    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));
    ImGui::Separator();

    //Disable the start buton if required
    ImGui::SetCursorScreenPos(ImVec2(canvas_p1.x * 1.05, ImGui::GetCursorScreenPos().y));

    //Disable the start button if there are no points in the canvas
    ImGui::BeginDisabled((cust_pnts.empty()) or cust_name.empty());

    //Start button
    if (ImGui::Button("Start")) {

        
        //Generate the points - and setup the test manager
        float X_POINT_BASE = -526;
        float Y_POINT_BASE = 1128;


        test_mgr.gen_trajectory.clear();
   
        for (int i = 0; i < cust_pnts.size(); i++) {
            //Translate and scale the points
            //BASED ON SANDBOX BOUNDS NOT CANVAS!!!

            float x_pos = X_POINT_BASE + ((cust_pnts[i].x - (CANVAS_SIZE * 0.1f)) * POINT_SCALING);
            float y_pos = Y_POINT_BASE + ( ((CANVAS_SIZE * 0.9f) - cust_pnts[i].y) * POINT_SCALING);

            test_mgr.gen_trajectory.push_back({ x_pos, y_pos, static_cast<float>(rob_height)});
            
            std::cout << "X: " << x_pos << " Y: " << y_pos << "\n";


        }


        //Update the robot info
        ABB_rob.set_speed(rob_speed);
        std::cout << "TEST\n";

        //Start the test
        test_mgr.set_data_path("C:/Users/User/Documents/Results/cust_test/" + cust_name);
        test_mgr.gen_test_title = cust_name;


        //Empty test name - autohandled
        test_mgr.test_selector("");     

    }


    ImGui::EndDisabled();
    

    //End the ImGui Loop
    ImGui::End();

    return;
}