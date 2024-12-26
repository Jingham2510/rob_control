#include "test_manager.hpp"


//Controls the robot to complete tests and saves the appropriate data
//Essentially a way of storing test scripts in the GUI 
//ENSURE THAT DATA PATH IS SET BEFORE RUNNING
test_manager::test_manager(ABB_tcp_client client) {

	//Set the robot that the tests are performed with
	robot = client;


    //DEFAULT DATA PATH - SHOULD BE MODIFIED
    data_path = "C:/Users/User/Documents/default.txt";

}

//Default constructor that does nowt
test_manager::test_manager() {

}


//Set the datapath
void test_manager::set_data_path(std::string new_path) {
    data_path = new_path;
}



//latency test
void test_manager::latency_test() {

	bool connected = false;


    std::vector<float> times;

    //ping 100 times
    for (int i = 0; i < 100; i++) {
        times.push_back((robot.ping()) * pow(10, -6));
    }

    //Save all the data to a text file 
    std::ofstream data_file(data_path);
    for (int i = 0; i < times.size(); i++) {
        data_file << times[i] << "\n";
    }

    data_file.close();

    return;
}


//First pass one movement test
void test_manager::first_pass_test() {

	std::vector<std::chrono::system_clock::time_point> time_data;
	//Stores both the force and posiiton data as a string
	//This is because we don't need to do any data analysis here
	std::vector<std::string> force_pos_data;
	std::string curr_force_pos;

	//TO BE DETERMINED - filler for now! - be very careful when doing it on the real one...
	float DES_X = 2200;
	float DES_Z = 190;
	std::vector<float> START_POS = { DES_X, -788, DES_Z };
	std::vector<float> END_POS = { DES_X, 315, DES_Z };
	//Steps based on distance and resolution of test
	int STEP_RES = 1;
	int dist_to_move = START_POS[1] - END_POS[1];
	enum pos_neg polarity = POSITIVE;

	//Ensure that the direction is correct
	if (START_POS[1] > END_POS[1]) {
		dist_to_move = -dist_to_move;
		polarity = NEGATIVE;
	}

	int steps = abs(dist_to_move * STEP_RES);
	std::vector<float> move_vector = {0, 2, 0};

	//Connect to the ABB robot
	//ABB_tcp_client client = ABB_tcp_client("192.168.125.1", 8888);

	bool connected = false;

	//Move the robot to the starting position - checked manually for now
	robot.set_joints({ -20.01, 58.59, 46.97, 4.81, 43.57, -32.28 });

	//Move the robot and log the data
	for (int i = 0; i <= steps; i++) {

		//Calculate the error from the desired pos
		move_vector = calc_line_err(robot.curr_pos, DES_X, START_POS[1] + (polarity * i * STEP_RES), DES_Z);

		//Move the robot - ensuring that the tool attempts to stay in a straight line
		curr_force_pos = robot.move_tool(move_vector);

		//Place the data in the arrays
		time_data.push_back(std::chrono::system_clock::now());

		force_pos_data.push_back(curr_force_pos);

		std::cout << curr_force_pos << "\n";
	}


	//Create the filename
	std::stringstream filename;
	filename << data_path << "placeholder" << "_first_pass.txt";

	//Save the data to a logfile
	std::ofstream data_file(filename.str());
	for (int i = 0; i < time_data.size(); i++) {
		//data_file << i << "," << time_data[i] << "," << force_pos_data[i] << "\n";
		data_file << i << "," << "PLACEHOLDER" << "," << force_pos_data[i] << "\n";
	}
	data_file.close();

	return;
}


//Calculates the robots position error from the desired error
std::vector<float> test_manager::calc_line_err(std::vector<float> curr_xyz, float desired_x, float desired_y, float desired_z) {

    //Create the vector float from the string
    std::vector<float> curr_xyz_vec = curr_xyz;

    //Create the vector from the differences (invert to ensure the tool moves the correct direction)
    std::vector<float> xyz_diff = { -(desired_x - curr_xyz_vec[0]), (desired_y - curr_xyz_vec[1]), -(desired_z - curr_xyz_vec[2]) };


    for (int i = 0; i < xyz_diff.size(); i++) {
        std::cout << "CURR: " << curr_xyz_vec[i] << " DIFF: " << xyz_diff[i] << "\n";
    }

    return xyz_diff;


}