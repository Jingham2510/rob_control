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
		float_storage.push_back(last_ping * pow(10, -6));
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
		for (int i = 0; i < float_storage.size(); i++) {
			data_file << float_storage[i] << "\n";
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
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, float_storage.size(), ImGuiCond_Always);
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.1, ImGuiCond_Always);
			ImPlot::PlotLine("Latency", count, &float_storage[0], float_storage.size());
			ImPlot::EndPlot();
		}
	}

}



//Moves to every coordinate given in the vector
//at a rate/speed determined by the step size
//Utilises TEST_FLAG_2 as it is designed to be used in another test
void test_manager::sequential_vertex_move(std::vector<std::vector<float>> vertexes, const float NO_OF_STEPS) {

	//The current force and position
	std::string curr_force_pos;

	//Move to the starting point (which is assumed as the first vertex)
	if (!TEST_FLAG_2) {

		//Move to above the starting point as to not disturb soil prematurely
		robot->set_pos({ vertexes[0][0], vertexes[0][1], vertexes[0][2] });

		//Move down into the soil
		robot->set_pos({ vertexes[0][0], vertexes[0][1], vertexes[0][2] });

		//Ensure the flag is set so it doesn't move back
		TEST_FLAG_2 = true;
	}
	//For each vertex (alreayd at starting point so start at 1)
	//Cant be a for loop - needs to be deconstructed into an if that runs multiple times
	if(TEST_FLAG_2 && !test_complete){

		//Fresh loop - calculate difference, direction and vectors for the stepsize
		if (loop_counter == 0) {



			//Calc difference in positions (alternatively based on point to point in the shape)
			std::vector<float> diff = {abs(vertexes[spare_counter][0] - vertexes[spare_counter - 1][0]),
									   abs(vertexes[spare_counter][1] - vertexes[spare_counter - 1][1]),
									   abs(vertexes[spare_counter][2] - vertexes[spare_counter - 1][2])
			};



			//XYZ - only 3 cartesian coordinate axes
			//Identfiy the directions that are required to be moved in each axis
			for (int i = 0; i < 3; i++) {

				//if target_pos > last target pos
				if (vertexes[spare_counter][i] >= vertexes[spare_counter - 1][i]) {
					seq_curr_xyz_dir[i] = POSITIVE;
				}
				else {
					seq_curr_xyz_dir[i] = NEGATIVE;
				}
			}
			//Print new target
			//std::cout << "NEW TARGET: " << " X: " << vertexes[spare_counter][0] << " Y: " << vertexes[spare_counter][1] << " Z: " << vertexes[spare_counter][2] << "\n";

			//Determine vector based on number of steps between each vertex
			//EQ is as follows - set direction * diff/no_of_steps
			seq_curr_mov_vec = { seq_curr_xyz_dir[0] * (diff[0]/NO_OF_STEPS), seq_curr_xyz_dir[1] * (diff[1]/NO_OF_STEPS), seq_curr_xyz_dir[2] * (diff[2] /NO_OF_STEPS)};

			//std::cout << "STEP VEC: " << seq_curr_mov_vec[0] << " Y: " << seq_curr_mov_vec[1] << " Z: " << seq_curr_mov_vec[2] << "\n";

		}

		//Stuff for movement
		std::vector<float> move_vector;


		//For the number of steps required - make the steps
		if (loop_counter <= NO_OF_STEPS) {

			//Calcualte the desired XYZ positions
			float DES_X = vertexes[spare_counter - 1][0] + (seq_curr_mov_vec[0] * loop_counter);
			float DES_Y = vertexes[spare_counter - 1][1] + (seq_curr_mov_vec[1] * loop_counter);
			float DES_Z = vertexes[spare_counter - 1][2] + (seq_curr_mov_vec[2] * loop_counter);

			//std::cout << "DES X: " << DES_X << " DES Y: " << DES_Y << " DES Z: " << DES_Z << "\n";

			//Calculate the error from the desired pos
			//Generate the movement vector
			move_vector = calc_line_err(robot->get_last_reported_pos(), DES_X, DES_Y, DES_Z);

			//Move the robot - ensuring that the tool attempts to stay in a straight line
			robot->move_tool(move_vector);
			
			//save the time data
			time_data.push_back(std::chrono::system_clock::now());
			
			//Save the force and position data
			storage_1.push_back(robot->get_last_reported_pos());
			storage_2.push_back(robot->get_last_reported_force());

			//Update loop counter
			loop_counter = loop_counter + 1;

			//Check whether all the steps have been done (and then reset the loop counter and increase the spare_counter)
			if (loop_counter == NO_OF_STEPS) {
				loop_counter = 0;
				spare_counter = spare_counter + 1;


				//Check position
				std::vector<float> curr_pos = robot->get_last_reported_pos();
				//std::cout << "ENDPOS: " << "X: " << curr_pos[0] << " Y: " << curr_pos[1] << " Z: " << curr_pos[2] << "\n";

			}		
		}

		//Do the plot
		force_displacement_plotting();

		//Check if spare counter has reached the limit
		if (spare_counter == vertexes.size()) {
			test_complete = true;
		}

	}

		
	//This function doesn't save the data nor end the test

	//This is because this is intended to be used inside 
	//another function which basically acts as a controller


}



void test_manager::gen_test() {
	
	//Create the window
	ImGui::Begin(gen_test_title.c_str());
	ImGui::Text(gen_test_title.c_str());

	//Setup the test
	if (!traj_sent) {
		//Send the trajectory
		for (int i = 0; i < gen_trajectory.size(); i++) {
			if (gen_trajectory[i].size() == 3) {
				robot->add_trans_traj_queue(gen_trajectory[i]);
			}
			else if (gen_trajectory[i].size() == 4) {
				robot->add_rot_traj_queue(gen_trajectory[i]);
			}
		}

		traj_sent = true;

		//Move the robot to above the starting point
		robot->set_pos({gen_trajectory[0][0], gen_trajectory[0][1], gen_trajectory[0][2] + 50});

		//Start the trajectory
		robot->traj_go();

		//Wait until the robot is moving to start reading
		while (robot->req_rob_mov()) {
			std::cout << "STUCK" << "\n";
		}
	}

	//Data acquisition 
	if (!test_complete && !TORQUE_ONLY) {
		log_traj_data();
	}
	//If its torque only measurements
	else if (!test_complete && TORQUE_ONLY) {
		
		//Get the time
		time_data.push_back(std::chrono::system_clock::now());
		//Get the robots torque
		storage_1.push_back(robot->req_torques());

		//Check if the test is complete
		if (robot->traj_done()) {
			std::cout << robot->rob_not_moving() << "\n";
			test_complete = true;
		}
	}


	if (test_complete && !file_saved && !TORQUE_ONLY) {
		//Save the data to a logfile
		std::ofstream data_file(data_path);
		for (int i = 0; i < time_data.size(); i++) {

			data_file << i << "," << time_data[i] << "," <<
				"[" << storage_1[i][0] << "," << storage_1[i][1] << "," << storage_1[i][2] << "],"
				<< "[" << storage_3[i][0] << "," << storage_3[i][1] << "," << storage_3[i][2] << "],"
				<< "[" << storage_2[i][0] << "," << storage_2[i][1] << "," << storage_2[i][2] << "," << storage_2[i][3] << "," << storage_2[i][4] << "," << storage_2[i][5] << "]" <<
				"\n";
		}

		data_file.close();

		file_saved = true;
	}

	else if (test_complete && !file_saved && TORQUE_ONLY) {
		//Save the data to a logfile
		std::ofstream data_file(data_path);
		for (int i = 0; i < time_data.size(); i++) {

			data_file << i << "," << time_data[i] << "," <<
				"[" << storage_1[i][0] << "," << storage_1[i][1] << "," << storage_1[i][2] << "," << storage_1[i][3] << "," << storage_1[i][4] << "," << storage_1[i][5] << "]"
				<< "\n";
		}

		data_file.close();

		file_saved = true;
	}


	//Display the test finished bit
	if (test_complete && file_saved) {

		//If not measuring just torque
		if (!TORQUE_ONLY) {
			force_displacement_plotting();
		}

		if (ImGui::Button("Close page")) {
			close = true;
			TEST_RUNNING_FLAG = false;
		}
	}


	ImGui::End();





}

//Logs the trajectory data
void test_manager::log_traj_data() {
	//Update the robot data
	robot->update_rob_info();

	//save the time data
	time_data.push_back(std::chrono::system_clock::now());

	//Save the force and position data
	storage_1.push_back(robot->get_last_reported_pos());
	storage_2.push_back(robot->get_last_reported_force());
	storage_3.push_back(robot->get_last_reported_ori());

	

	//Check if the test is complete
	if (robot->traj_done()) {
		std::cout << robot->rob_not_moving() << "\n";
		test_complete = true;
	}

	//Do the plot
	force_displacement_plotting();
}


//Generic plotting for force/displacement/xyzerr/
//TODO: make more generic by adding label and allowing any extra data to be used not just error
void test_manager::force_displacement_plotting() {

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
	///static RollingBuffer xerr_data;
	///static RollingBuffer yerr_data;
	///static RollingBuffer zerr_data;


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

		///xerr_data.Span = 100;
		///yerr_data.Span = 100;
		//zerr_data.Span = 100;


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

		/*
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
		*/




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


	//Check the direction of the movement vectors
	float x_mov, y_mov, z_mov;

	//Based on tool orientation
	x_mov =  (desired_x - curr_xyz[0]);

	y_mov = desired_y - curr_xyz[1];

	z_mov = (desired_z - curr_xyz[2]);




    //Create the vector from the differences (invert to ensure the tool moves the correct direction)
    std::vector<float> xyz_diff = {x_mov, y_mov, z_mov};



    return xyz_diff;


}


//Selects a test (and threads it hopefully)
void test_manager::test_selector(std::string test_name) {


	//Inside if statement generate trajectory and set title
	//Then just set test running and the main loop uses the generic trajectory command

	if (test_name == "latency_test") {
		//Prep storage for test data
		float_storage.clear();

		LATENCY_TEST_FLAG = true;
	}


	if (test_name == "first_pass_test") {
		gen_test_title = "First pass";

		float DES_X = 262;
		float DES_Z = 85;

		std::vector<float> START_POS = { DES_X, 1650, DES_Z };
		std::vector<float> END_POS = { DES_X, 2550, DES_Z };

		gen_trajectory = { {DES_X, 1650, 150}, START_POS, END_POS, START_POS };
		

	}

	if (test_name == "tri_poly_test") {
		gen_test_title = "Triangle";

		//Define the three vertexes
		std::vector<float> P1 = { 209, 1787, 100 };
		std::vector<float> P2 = { -108, 2167, 100 };
		std::vector<float> P3 = { 680, 2600, 100 };

		gen_trajectory = { P1, P2, P3, P1 };

	}

	if (test_name == "circle_test") {

		gen_test_title = "Circle";

		//Define the centre point - needs to ve verified
		std::vector<float> centre = { 200, 2160, 100 };

		int N = 3;
		int radius = 350;


		gen_trajectory.clear();
		for (int i = 0; i <= 360 * N; i++) {
			gen_trajectory.push_back({ centre[0] + float((sin(i * (3.184 / 180)) * radius)) , centre[1] + float(cos(i * (3.184 / 180)) * radius), centre[2] });
		}

	}

	if (test_name == "spiral_test") {
		gen_test_title = "Spiral";
		//Define the centre point - needs to ve verified
		std::vector<float> centre = { 200, 2160, 100 };

		double start_r = 100;
		double stop_r = 250;
		int N = 5;

		double curr_r = start_r;

		for (float i = 0; i <= 360 * N; i++) {
			//Calculate the next point
			gen_trajectory.push_back({ centre[0] + float((sin(i * (3.184 / 180)) * curr_r)) , centre[1] + float(cos(i * (3.184 / 180)) * curr_r), centre[2] });

			//Calculate the radius of the spiral
			//starting radius + percentage completion of the drawing
			if (start_r <= stop_r) {
				curr_r = start_r + (abs(stop_r - start_r) * (i / (360 * N)));
			}
			else {
				curr_r = start_r - (abs(stop_r - start_r) * (i / (360 * N)));
			}
		}
	}

	if (test_name == "point_test") {
		gen_test_title = "Point loading";

		//Define the starting point
		std::vector<float> start_point = { 200, 2160, 160 };

		//Define the point to load
		std::vector<float> load_point = { 200, 2160, 100 };

		int N = 3;
	
		gen_trajectory.clear();
		for (int i = 0; i < N; i++) {
			gen_trajectory.push_back(start_point);
			gen_trajectory.push_back(load_point);
		}
		

	}


	if (test_name == "ori_frame_test") {
		gen_test_title = "Rotation test";

		//CHECK THESE
		
		//Define the starting point of the first line 
		std::vector<float> start_point = { -60, 1750, 250 };
		//Define the point of contact
		std::vector<float> contact_point = { -60, 1750, 85 };
		//Define the end point of the first line 
		std::vector<float> end_point = { -60, 2450, 85 };
		
		//Define the rotation
		std::vector<float> rot = { 0, 0, 0 };


		//Create the trajectory
		for (int i = 0; i < 3; i++) {
	
			//Add start point
			gen_trajectory.push_back(start_point);
			
			//Rotate end-effector
			gen_trajectory.push_back(quart_to_float(euler_to_q(rot)));

			//Add contact point
			gen_trajectory.push_back(contact_point);

			//Add end point
			gen_trajectory.push_back(end_point);
			gen_trajectory.push_back({ end_point[0], end_point[1], end_point[2] + 85 });


			//Update points
			start_point = {start_point[0] + 250, start_point[1], start_point[2]};
			contact_point = { contact_point[0] + 250, contact_point[1], contact_point[2] };
			end_point = { end_point[0] + 250, end_point[1], end_point[2] };

			//Update the orientation
			rot = { rot[0] + deg_to_rad(90) , rot[1], rot[2] };



		}
		

	}


	//Torque measurement test
	if (test_name == "torque_test") {
		gen_test_title = "Torque Measurement";

		//Create the start and end point
		//Add them to the generatred trajectory
		gen_trajectory.push_back({ 262, 1650, 200 });
		gen_trajectory.push_back({262, 1650, 85});
		gen_trajectory.push_back({262, 1850, 85});
		gen_trajectory.push_back({ 262, 1650, 150 });

		//Set the torque only flag
		TORQUE_ONLY = TRUE;

	}

	else {
		TORQUE_ONLY = FALSE;
	}


	//Reset test variables
	// 
	//Setup the test
	traj_sent = false;
	test_complete = false;
	file_saved = false;
	close = false;
	loop_counter = 0;


	//Prep storage for the test data
	storage_1.clear();
	storage_2.clear();
	storage_3.clear();
	test_trajectory.clear();

	time_data.clear();

	//Setup the test flags

	TEST_FLAG_1 = false;
	TEST_FLAG_2 = false;

	TEST_RUNNING_FLAG = true;



	}
	
	


