#include <winsock2.h>
#include<WS2tcpip.h>
#include <string>
#include <vector>

#ifndef ABB_TCP_CLIENT 
#define ABB_TCP_CLIENT


//Relevant winsock library
#pragma comment(lib, "ws2_32.lib")

//Definition of the ABB-http client 
class ABB_tcp_client{
    public:
        //Creates the http client and auto connects to the socket
        ABB_tcp_client(const char *, int);

        //connection test - sends an echo request to the server and times it 
        float ping();

        //Set the joints of the ABB robot
        int set_joints(std::vector<float>);

        //Move the tool relative to it's current position
        std::string move_tool(std::vector<float>);

        //Get the robots current xyz pos
        std::string get_xyz();


        //Closes the connection
        void close_connection();

    private:
        //Windows socket data
        WSADATA wsa;
        //ABB server socket
        SOCKET sock;

        //ABB server IP
        const char *ip;
        //ABB server port
        int port;

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


};

#endif