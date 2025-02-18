#include "ABB_tcp_client.hpp"
#include<iostream>
#include<chrono>
#include<sstream>




ABB_tcp_client::ABB_tcp_client() {

}


//Creates and connects to the ABB http socket
ABB_tcp_client::ABB_tcp_client(const char *client_ip, int client_port, bool *connect_flag){

    ip = client_ip;
    port = client_port;
    connected = connect_flag;



    //Only inisitalise WSA once
    if (!wsa_flag) {

        //Initialise the WSA routines
        std::cout << "Starting client..." << "\n";

        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            std::cout << "Startup failed. ERR: " << WSAGetLastError() << "\n";
            *connected = false;
        }

        else {
            wsa_flag = true;
            std::cout << "Client started" << "\n";
        }
        
    }

    

    //If the robot successfully connects
    if(wsa_flag){        

        *connect_flag = true;
        

        if (!sock_flag)
        {
            //Create the socket
            create_sock();
        }

        //Only try to connect if the socket connected
        if (*connected) {
            //Connect to the server
            connect_to_ABB();
        }
    }



}




//Creates a socket in the http client object
void ABB_tcp_client::create_sock(){

    std::cout << "Creating socket" << "\n";
    
    //Attempt to create the socket
    if((sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET){

        std::cout << "Socket Creation Failed: " << WSAGetLastError();
        *connected = false;

    }

    else{
        sock_flag = true;
        std::cout << "Socket created" << "\n";
    }
}


//Connect toe the ABB http host
void ABB_tcp_client::connect_to_ABB(){

    std::cout << "Connecting to server..." << "\n";

    //Setup the server struct
    ABB_server.sin_addr.s_addr = inet_addr(ip);
    //InetPton(AF_INET, ip, &ABB_server.sin_addr.s_addr);
    ABB_server.sin_family = AF_INET;    
    ABB_server.sin_port = htons(8888);


    //Attempt to connect to the socket
    if(connect(sock, (struct sockaddr*) &ABB_server, sizeof(ABB_server)) < 0){

        std::cout << "Connection error: " << WSAGetLastError() << "\n";
        *connected = false;

    }
    else{
        std::cout<< "Connected to: " << ip << "\n";

        //Update the robots info
        update_rob_info();

    }

}

//Closes the connection
void ABB_tcp_client::close_connection(){

    request("CLOS:1");
    closesocket(sock);
    WSACleanup();

}

//Send a request to the server
int ABB_tcp_client::request(std::string request){

    //Convert string to old C type for winsock compatibility
    const char *req = request.data();

    //Attempt to send the request
    if(send(sock, req, strlen(req), 0) < 0){

        std::cout << "ERR: send failed - " << WSAGetLastError() << "\n";
        return -1;
    }


    return 1;
}

//Recieve a response from the server
std::string ABB_tcp_client::recieve(){

    //Buffer to store the server reply
    char buff[2000];
    int recv_err;
    std::string response;

    //Attempt to read a reply from the socket
    if ((recv_err = recv(sock, buff, 2000, 0)) == SOCKET_ERROR){
        std::cout << "Err: socket read - " << WSAGetLastError() << "\n";
        return "SOCKERROR";
    }
    else{
        response = buff;
    }

    //Convert the c-string to a c++ string - and strip the garbage
    response = response.substr(0, response.find("!"));

    //std::cout << response << "\n";

    return response;
}

//Connection test
float ABB_tcp_client::ping(){

    //Auto used here because the type is ridiculously long
    //mark the time
    auto start = std::chrono::high_resolution_clock::now();

    //Complete the ping
    request("ECHO:PING");

    //Receive the response
    std::string response = recieve();

    //Mark the finish time
    auto stop = std::chrono::high_resolution_clock::now();

  
    auto exe_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << response << " - Ping complete - Time taken: " << exe_time.count() << "us\n";

    return exe_time.count();

}

//Set the joints to a specific angle
int ABB_tcp_client::set_joints(std::vector<float> jnt_angs){

    //The string stream to create the command
    std::stringstream stream;

    //Check there are the correct number of joint angles    
    if (jnt_angs.size() != 6){
        std::cout << "ERR: incorrect number of joint angles!";
        return -1;
    }

    //A bit hacky but creates the set joint command with the correct format
    //No need to turn external axes the robot is set
    stream << "STJT:[[" << com_vec_to_string(jnt_angs) << "], [9E9,9E9,9E9,9E9,9E9,9E9]]";


    //Send the set joint command to the ABB server
    request(stream.str());

    //Print the response from the server
    std::cout << recieve() << "\n";


    update_rob_info();

  

    return 1;
}

//Set the tools orientation without moving the robots arm 
//specified in quartenions
void ABB_tcp_client::set_ori(std::vector<float> des_ori) {

    //Create the cmd string setup
    std::stringstream cmd;
    float quart_check = 0;

    //Check there are the right number of entries
    if (des_ori.size() != 4) {
        std::cout << "ERR: INVALID ORIENTATION - SIZE" << "\n";
        return;

    }

    //Check that the quartenions are valid (i.e. they all square to add to 1)
    for (int i = 0; i < des_ori.size(); i++) {
        quart_check = quart_check + pow(des_ori[i], 2);
    }

    std::cout << quart_check << "\n";

    if (!(quart_check > 0.99 and quart_check < 1.01)) {
        std::cout << "ERR: INVALID ORIENTATION - VALUE != 1" << "\n";
        return;
    }

    //Send the request
    cmd << "STOR:[" << com_vec_to_string(des_ori) << "]";

    request(cmd.str());


    //Wait for the okay message
    //blocking
    recieve();

    //Update the robots position force etc
    update_rob_info();

    return;

}


//Move the tool relative to its current position
void ABB_tcp_client::move_tool(std::vector<float> xyz){

    //The commmand and the command constructor
    std::stringstream cmd_stream;


    //Check the xyz count is correct
    if(xyz.size() != 3){
        std::cout << "ERR: Incorrect number of coords supplied" << "\n";
        return;
    }

    std::cout << "MOVE TL REQ" << "\n";

    //Send move commands
    cmd_stream << "MVTL:[" << com_vec_to_string(xyz) << "]";

    request(cmd_stream.str());

    //Wait until move is done
    recieve();

    //request robot info
    update_rob_info();

    
    return;
}

//Set the robots position by specifying the XYZ coordinates
//The robot will keep the rotaiton orientiation it was originally at
void ABB_tcp_client::set_pos(std::vector<float> xyz) {

    //The commmand constructor
    std::stringstream cmd_stream;



    //Check the xyz count is correct
    if (xyz.size() != 3) {
        std::cout << "ERR: Incorrect number of coords supplied" << "\n";
        return;
    }

    //Create the command and request it 
    cmd_stream << "MVTO:[" << com_vec_to_string(xyz) << "]";

    request(cmd_stream.str());

    //Wait to recieve the response to say it is done
    if(recieve() == "MVTO OK");



    update_rob_info();


    return;

}

//Add to the trajectory queue of the robot
void ABB_tcp_client::add_to_traj_queue(std::vector<float> xyz) {

    //The commmand constructor
    std::stringstream cmd_stream;



    //Check the xyz count is correct
    if (xyz.size() != 3) {
        std::cout << "ERR: Incorrect number of coords supplied" << "\n";
        return;
    }

    //Create the command and request it 
    cmd_stream << "TQAD:[" << com_vec_to_string(xyz) << "]";

    request(cmd_stream.str());


    return;
}

//Start the trajectory queue
void ABB_tcp_client::traj_go() {
    request("TJGO:1");
}
//Stop the trajectory queue
void ABB_tcp_client::traj_stop() {
    request("TJST:0");
}

bool ABB_tcp_client::traj_queue_empty() {
    request("TJQE:?");

    //Turn recieved string into boolean
    bool empty = true;

    if (recieve() == "FALSE") {
        empty = false;
    }   

    return empty;
}



std::vector<float> ABB_tcp_client::req_xyz() { 

    //Send the getpos command
    request("GTPS:0");

    //save the current position
    std::vector<float> pos = xyz_str_to_float(recieve());

    return pos;

}

std::vector<float> ABB_tcp_client::req_ori() {

    //Send the get orientation command
    request("GTOR:0");

    std::vector<float> orientation = xyz_str_to_float(recieve());

    return orientation;
}


std::vector<float> ABB_tcp_client::req_jnt_angs() {

    //Send the get joint angles command
    request("GTJA:0");

    std::vector<float> joint_angs = xyz_str_to_float(recieve());

    return joint_angs;


}


std::vector<float> ABB_tcp_client::req_force() {
    //Send the get force command
    request("GTFC:0");

    std::vector<float> force = xyz_str_to_float(recieve());

    return force;

}


bool ABB_tcp_client::req_rob_mov() {

    //Send the request moving state command
    request("MVST:0");

    std::istringstream(recieve()) >> not_moving_flag;

    return not_moving_flag;


}

std::vector<float> ABB_tcp_client::get_last_reported_pos() {
    return curr_pos;
}

std::vector<float> ABB_tcp_client::get_last_reported_ori() {
    return curr_ori;
}

std::vector<float> ABB_tcp_client::get_last_reported_jnt_angs() {
    return curr_jnt_angs;
}

std::vector<float> ABB_tcp_client::get_last_reported_force() {
    return curr_force;
}



std::string ABB_tcp_client::get_model() {
    //Send the model request command
    request("RMDL:0");

    std::string model = recieve();


    return model;
}


//Updates all of the roobt info via request
void ABB_tcp_client::update_rob_info() {

    //Update position
    curr_pos = req_xyz();

    //Update force
    curr_force = req_force();


    //Request the robots orientation
    curr_ori = req_ori();

    //Request the robots joint angles
    curr_jnt_angs = req_jnt_angs();


    //Request the robots motion sate
    not_moving_flag = req_rob_mov();


    return;

}


//Takes in a vector of floats then returns it as a string - comma seperated
std::string ABB_tcp_client::com_vec_to_string(std::vector<float> data){

    std::stringstream stream;

    //Goes through the whole array and adds it
    for(size_t i = 0; i <data.size(); i++){
     
        stream << data[i];
        
        if(i != data.size() - 1){
            stream <<  ",";
        }

    }

    return stream.str();

}

//Turns the RAPID xyz string to a vector
std::vector<float> ABB_tcp_client::xyz_str_to_float(std::string xyz) {

     
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