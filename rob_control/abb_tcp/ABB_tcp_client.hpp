#pragma once
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#include <string>
#include <vector>

//Relevant winsock library
#pragma comment(lib, "ws2_32.lib")


#ifndef ABB_TCP_CLIENT 
#define ABB_TCP_CLIENT



//Definition of the ABB-http client 
class ABB_tcp_client{
    public:

        //default
        ABB_tcp_client();

        //Creates the http client and auto connects to the socket
        ABB_tcp_client(const char *, int, bool *);

        //connection test - sends an echo request to the server and times it 
        float ping();

        //Set the joints of the ABB robot
        int set_joints(std::vector<float>);

        //Move the tool relative to it's current position
        void move_tool(std::vector<float>);

        //Set the robots position - blocking
        void set_pos(std::vector<float>);

        //Get the robots current xyz pos
        std::vector<float> req_xyz();

        //Get the robots current ZYX euler orientation
        std::vector<float> req_ori();

        //Get the robots joint angles
        std::vector<float> req_jnt_angs();

        //Get the robots currently reported force
        std::vector<float> req_force();

        //Get the last reported pos
        std::vector<float> get_last_reported_pos();

        //Get the last reported orientation
        std::vector<float> get_last_reported_ori();

        //Get the last reported set of joint angles
        std::vector<float>get_last_reported_jnt_angs();


        //Get the last reported force
        std::vector<float> get_last_reported_force();


        //Get the robots model
        std::string get_model();


        //Closes the connection
        void close_connection();



    private:



        //FLAGS

        //Connection flag (supplied externally)
        bool* connected;

        //startup flags
        bool wsa_flag = false;
        bool sock_flag = false;




        //Windows socket data
        WSADATA wsa;
        //ABB server socket
        SOCKET sock;

        //ABB server IP
        const char *ip;
        //ABB server port
        int port;

        //Robots current position
        std::vector<float> curr_pos;

        //Robots current orientation
        std::vector<float> curr_ori;

        //Robots current 6-axis force
        std::vector<float> curr_force;

        //Robots current joint angles
        std::vector<float> curr_jnt_angs;

        //The server struct 
        struct sockaddr_in ABB_server;
        //Creates a socket
        void create_sock();

        //Connects to the ABB client for the given ip
        void connect_to_ABB();

        //Send a string of bytes
        int request(std::string);

        //Recieve a string of bytes
        std::string recieve();    

        //Helper function to create strings from vectors of floats
        std::string com_vec_to_string(std::vector<float>);

        

        std::vector<float> xyz_str_to_float(std::string xyz);


};

#endif