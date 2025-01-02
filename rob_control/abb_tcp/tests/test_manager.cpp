#include "test_manager.hpp"


//Controls the robot to complete tests and saves the appropriate data
//Essentially a way of storing test scripts in the GUI 
//ENSURE THAT DATA PATH IS SET BEFORE RUNNING
test_manager::test_manager(ABB_tcp_client *client) {

	//Set the robot that the tests are performed with
	//Set to the address of the robot alreayd being used
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

//Return the current datapath
std::string test_manager::get_data_path() {
	return data_path;
}

//Report the test running status
bool test_manager::test_running(){
	return TEST_RUNNING_FLAG;
}


//Latency test
void test_manager::latency_test() {

	float last_ping;

	ImGui::Begin("Latency Test");
	ImGui::Text("Latency Test");

	//Create the graph displaying the info

	//Only do the next part of the test when required
	//Complete one loop at a time
	if (!test_complete) {
		last_ping = robot->ping();
		//Complete a ping
		storage_1.push_back(last_ping * pow(10, -6));
		loop_counter++;
		if (loop_counter == 99) {
			test_complete = true;
		}
		latency_plotting(last_ping);
	}

	

	//Save the file once the test is complete - all in one go
	if (test_complete && !file_saved) {

		//Save all the data to a text file 
		std::ofstream data_file(data_path);
		for (int i = 0; i < storage_1.size(); i++) {
			data_file << storage_1[i] << "\n";
		}

		data_file.close();
		file_saved = true;
	}


	//If the test is complete
	if (test_complete && file_saved) {

		//Display the complete graph - doesnt matter what this variable is 
		latency_plotting(0);

		//Display the close button
		if (ImGui::Button("Close Page")) {
			close = true;
			LATENCY_TEST_FLAG = false;
			TEST_RUNNING_FLAG = false;
		}
	}

	ImGui::End();	
		
    return;
}

//Live latency test plotting
void test_manager::latency_plotting(float next_point) {

	//Create the rolling plot while the test is happening
	if (!test_complete) {

		//Setup the buffer
		static RollingBuffer latency_data;
		static float t = 0;
		latency_data.Span = 100.0f;
		//Get the time
		t += ImGui::GetIO().DeltaTime;
		

		//Add the point to the buffer
		latency_data.AddPoint(t, next_point);

		//Determine plot flags
		static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

		//Create the plot
		if (ImPlot::BeginPlot("##Latency")) {
			ImPlot::SetupAxes("Time","Latency", flags, flags);
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, 10);
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 20000);
			ImPlot::PlotLine("Latency (ms)", &latency_data.Data[0].x, &latency_data.Data[0].y, latency_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::EndPlot();
		}

		

	}
	//If test has stopped running - no need for rolling buffer
	//Plot static plot
	else {
		
		static float count[100];

		for (int i = 0; i < 100; i++) {
			count[i] = i;
		}


		if (ImPlot::BeginPlot("##Latency")) {
			ImPlot::SetupAxes("Iteration", "Latency");
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, storage_1.size(), ImGuiCond_Always);
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.1, ImGuiCond_Always);
			ImPlot::PlotLine("Latency", count, &storage_1[0], storage_1.size());
			ImPlot::EndPlot();
		}
	}

}


//First pass one movement test
void test_manager::first_pass_test() {



	ImGui::Begin("First Pass");
	ImGui::Text("First Pass Test");


	//Store the current force position
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
	std::vector<float> move_vector;


	if (!TEST_FLAG_1) {
		//Move the robot to the starting position - checked manually for now
		robot->set_joints({ -20.01, 58.59, 46.97, 4.81, 43.57, -32.28 });
		TEST_FLAG_1 = true;
	}


	//Move the robot and log the data
	if (TEST_FLAG_1 && !test_complete) {



		//Calculate the error from the desired pos
		move_vector = calc_line_err(robot->get_last_reported_pos(), DES_X, START_POS[1] + (polarity * loop_counter * STEP_RES), DES_Z);

		//Move the robot - ensuring that the tool attempts to stay in a straight line
		curr_force_pos = robot->move_tool(move_vector);

		//Place the data in the arrays
		time_data.push_back(std::chrono::system_clock::now());

		string_storage.push_back(curr_force_pos);

		
		loop_counter++;

		if (loop_counter == (steps - 1)) {
			test_complete = true;
		}
	}

	force_displacement_plotting( move_vector);

	if (test_complete && !file_saved) {

	

		//Save the data to a logfile
		std::ofstream data_file(data_path);
		for (int i = 0; i < time_data.size(); i++) {
			
			data_file << i << "," << time_data[i] << "," << string_storage[i] << "\n";
			
		
		
		}
		data_file.close();

		file_saved = true;
	}

	

	//Display the test finished bit
	if (test_complete && file_saved) {
		if(ImGui::Button("Close page")){
			close = true;
			FIRST_PASS_FLAG = false;
			TEST_RUNNING_FLAG = false;
		}
	}


	ImGui::End();

	return;
}

//Generic plotting for force/displacement/xyzerr/
//TODO: make more generic by adding label and allowing any extra data to be used not just error
void test_manager::force_displacement_plotting(std::vector<float> xyz_err) {

	//TODO: -4 plots each contianign a set number of lines
	//Plot orientation graph (requires a get orientation cmd) ~ ABB doesnt report it...
	//Show plot when finished

	//XYZ position buffers
	static RollingBuffer xpos_data;
	static RollingBuffer ypos_data;
	static RollingBuffer zpos_data;

	//6-axis force buffers
	static RollingBuffer xforce_data;
	static RollingBuffer yforce_data;
	static RollingBuffer zforce_data;
	static RollingBuffer Rxforce_data;
	static RollingBuffer Ryforce_data;
	static RollingBuffer Rzforce_data;


	//Error buffers
	static RollingBuffer xerr_data;
	static RollingBuffer yerr_data;
	static RollingBuffer zerr_data;


	//Determine plot flags
	static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;



	if (!test_complete) {
		
		//Setup the buffers
		xpos_data.Span = 100;
		ypos_data.Span = 100;
		zpos_data.Span = 100;

		xforce_data.Span = 100;
		yforce_data.Span = 100;
		zforce_data.Span = 100;
		Rxforce_data.Span = 100;
		Ryforce_data.Span = 100;
		Rzforce_data.Span = 100;

		xerr_data.Span = 100;
		yerr_data.Span = 100;
		zerr_data.Span = 100;


		static float t = 0;

		//Get the time
		t += ImGui::GetIO().DeltaTime;

		//Get the position data
		std::vector<float> last_pos = robot->get_last_reported_pos();

		//Add the points to the buffer
		xpos_data.AddPoint(t, last_pos[0]);
		ypos_data.AddPoint(t, last_pos[1]);
		zpos_data.AddPoint(t, last_pos[2]);

		//Create the plot
		if (ImPlot::BeginPlot("Position")) {
			ImPlot::SetupAxes("Time", "Position", flags, flags);
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, 100);
			ImPlot::SetupAxisLimits(ImAxis_Y1, -2500, 2500);
			ImPlot::PlotLine("X pos", &xpos_data.Data[0].x, &xpos_data.Data[0].y, xpos_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Y pos", &ypos_data.Data[0].x, &ypos_data.Data[0].y, ypos_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Z pos", &zpos_data.Data[0].x, &zpos_data.Data[0].y, zpos_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::EndPlot();
		}

		//Get the force data
		std::vector<float> last_force = robot->get_last_reported_force();

		//Add the points to the force buffers
		xforce_data.AddPoint(t, last_force[0]);
		yforce_data.AddPoint(t, last_force[1]);
		zforce_data.AddPoint(t, last_force[2]);
		Rxforce_data.AddPoint(t, last_force[3]);
		Ryforce_data.AddPoint(t, last_force[4]);
		Rzforce_data.AddPoint(t, last_force[5]);

		//Plot the 6 axis forces
		if (ImPlot::BeginPlot("Force")) {
			ImPlot::SetupAxes("Time", "Force", flags, flags);
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, 100);
			ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 10);
			ImPlot::PlotLine("X", &xforce_data.Data[0].x, &xforce_data.Data[0].y, xforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Y", &yforce_data.Data[0].x, &yforce_data.Data[0].y, yforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Z", &zforce_data.Data[0].x, &zforce_data.Data[0].y, zforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("RX", &Rxforce_data.Data[0].x, &Rxforce_data.Data[0].y, Rxforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("RY", &Ryforce_data.Data[0].x, &Ryforce_data.Data[0].y, Ryforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("RZ", &Rzforce_data.Data[0].x, &Rzforce_data.Data[0].y, Rzforce_data.Data.size(), 0, 0, 2 * sizeof(float));

			ImPlot::EndPlot();
		}


		//Add the points to the error buffers
		xerr_data.AddPoint(t, xyz_err[0]);
		yerr_data.AddPoint(t, xyz_err[1]);
		zerr_data.AddPoint(t, xyz_err[2]);

		//Plot the error data
		//Create the plot
		if (ImPlot::BeginPlot("Errors")) {
			ImPlot::SetupAxes("Time", "Error", flags, flags);
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, 100);
			ImPlot::SetupAxisLimits(ImAxis_Y1, -5, 5);
			ImPlot::PlotLine("X err", &xerr_data.Data[0].x, &xerr_data.Data[0].y, xerr_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Y err", &yerr_data.Data[0].x, &yerr_data.Data[0].y, yerr_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Z err", &zerr_data.Data[0].x, &zerr_data.Data[0].y, zerr_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::EndPlot();
		}




	}
	//Display the completed test
	//For this we only want to display the 6-axis force 
	//As the data will eventually just get processed
	else {


		//Plot the 6 axis forces
		if (ImPlot::BeginPlot("Force")) {
			ImPlot::SetupAxes("Time", "Force", flags, flags);
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, 100);
			ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 10);
			ImPlot::PlotLine("X", &xforce_data.Data[0].x, &xforce_data.Data[0].y, xforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Y", &yforce_data.Data[0].x, &yforce_data.Data[0].y, yforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("Z", &zforce_data.Data[0].x, &zforce_data.Data[0].y, zforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("RX", &Rxforce_data.Data[0].x, &Rxforce_data.Data[0].y, Rxforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("RY", &Ryforce_data.Data[0].x, &Ryforce_data.Data[0].y, Ryforce_data.Data.size(), 0, 0, 2 * sizeof(float));
			ImPlot::PlotLine("RZ", &Rzforce_data.Data[0].x, &Rzforce_data.Data[0].y, Rzforce_data.Data.size(), 0, 0, 2 * sizeof(float));

			ImPlot::EndPlot();
		}


	}




}




//Calculates the robots position error from the desired error
std::vector<float> test_manager::calc_line_err(std::vector<float> curr_xyz, float desired_x, float desired_y, float desired_z) {

    //Create the vector float from the string
    //std::vector<float> curr_xyz_vec = curr_xyz;

    //Create the vector from the differences (invert to ensure the tool moves the correct direction)
    std::vector<float> xyz_diff = { -(desired_x - curr_xyz[0]), (desired_y - curr_xyz[1]), -(desired_z - curr_xyz[2]) };


    for (int i = 0; i < xyz_diff.size(); i++) {
        std::cout << "CURR: " << curr_xyz[i] << " DIFF: " << xyz_diff[i] << "\n";
    }

    return xyz_diff;


}


//Selects a test (and threads it hopefully)
void test_manager::test_selector(std::string test_name) {


	//One giant if statement to select the test
	
	if (test_name == "latency_test"){	

		//Setup the test
		test_complete = false;
		file_saved = false;
		close = false;
		loop_counter = 0;

		//Prep storage for test data
		storage_1.clear();

		//Setup the test flags
		TEST_RUNNING_FLAG = true;
		LATENCY_TEST_FLAG = true;
	}

	if (test_name == "first_pass_test") {

		//Setup the test
		test_complete = false;
		file_saved = false;
		close = false;
		loop_counter = 0;

		//Prep storage for the test data;
		string_storage.clear();
		time_data.clear();

		//Setup the test flags
		TEST_FLAG_1 = false;

		TEST_RUNNING_FLAG = true;
		FIRST_PASS_FLAG = true;
	}

	

}


