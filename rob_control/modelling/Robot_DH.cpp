#include"Robot_DH.hpp"




RobotDH::RobotDH() {

}


RobotDH::RobotDH(std::vector<DHLink> links, std::vector<std::string> offsets, std::vector<int> invs){
    

    n_of_links = links.size();
    
    //Ensure that the lists are the same sizes
    if (n_of_links != offsets.size()){
        std::cout << "WARNING: Setup vectors are different lengths";
        
    }

    //Assign all the parameters
    link_list = links;

    offset_config = offsets;

    inv_vec = invs;
   
   //Update the coupled offsets
    update_link_offsets();

    //Update the position and orientation
    update_pos_orient();
    
}

//Constructor when a model is provided
RobotDH::RobotDH(std::string model_name) {

    valid_model = false;
    int model_pos;

    //See if the model is valid
    for (int i = 0; i < VALID_MODELS.size(); i++) {
        if (model_name == VALID_MODELS[i]) {
            valid_model = true;
            model_pos = i;
            break;
        }
    }


    //Check that the model is valid
    if (valid_model) {        

        //std::cout << "Valid model: " << model_pos << "\n";
        
        //Load the models info
        std::ifstream file("models/" + MODEL_FILENAMES[model_pos]);
        std::string curr_line;

        //check the number of links - stored on the first line 
        getline(file, curr_line);

        n_of_links = std::stoi(curr_line);

        //list contianing the link info
        std::vector<float> link_info;

        //Used to help split the string
        size_t str_pos;
        std::string token;
        std::string delimiter = ",";

        //Iterate through N link lines
        for (int i = 0; i < n_of_links; i++) {
            //Clear the link info
            link_info.clear();


            //Get the link info from the line
            getline(file, curr_line);

            //std::cout << curr_line << "\n";
            
            //Split the link info by the delimiter ","
            str_pos = 0;
            while ((str_pos = curr_line.find(delimiter)) != std::string::npos) {
                token = curr_line.substr(0, str_pos);
                link_info.push_back(std::stof(token));
                curr_line.erase(0, str_pos + delimiter.length());
            }

            //Add the last token 
            link_info.push_back(std::stof(curr_line));

            //Create link and put it in the link list
            link_list.push_back(DHLink(link_info[0], link_info[1], link_info[2], link_info[3], link_info[4]));
        }

        //Read the coupling offset config
        getline(file, curr_line);
        
        //Split the coupling offset by the delimiter ","
        str_pos = 0;
        while ((str_pos = curr_line.find(delimiter)) != std::string::npos) {
            token = curr_line.substr(0, str_pos);
            offset_config.push_back(token);
            curr_line.erase(0, str_pos + delimiter.length());
        }
        
        
        //Get the inverter config
        getline(file, curr_line);
        str_pos = 0;
        while ((str_pos = curr_line.find(delimiter)) != std::string::npos) {
            token = curr_line.substr(0, str_pos);
            inv_vec.push_back(std::atoi(token.c_str()));
            curr_line.erase(0, str_pos + delimiter.length());            
        }


        
        //Close the file
        file.close();




        //Update the links and positions 
        update_link_offsets();

        update_pos_orient();


    }

    else if (model_name == "NA") {
        //DO NOTHING - INITIALISING
    }

    else {
        std::cout << "WARNING: INVALID MODEL" << "\n";
    }



}


//Calcualtes the transform matrix from the current links and joint angles
void RobotDH::calc_transform(){
   
    //Reset the transform matrix to the first transform
    trans_mat = link_list.at(0).get_hg();   

    

    //Iterate through the links   - start one later
    for (int i = 1; i < n_of_links; i++){
       
        trans_mat = trans_mat * link_list.at(i).get_hg();
                
    }
}


//Return the current transformation matrix
Matrix4f RobotDH::get_trans(){
    return trans_mat;
}


//Updates the models joints with a set of provided joint angles
void RobotDH::update_joints(std::vector<float> joint_angs) {

    //Check that their are the correct number of joint angles
    if (joint_angs.size() != n_of_links) {
        std::cout << "DEBUG: incorrect no of angles!";
        return;
    }

    
    //Update the thetas
    for (int i = 0; i < n_of_links; i++) {

        std::cout << "link " << i << " : " << link_list[i].get_theta() << "\n";
        std::cout << "inp " << i << " : " << joint_angs[i] << "\n";
        std::cout << "inverse? " << inv_vec[i] << "\n";

        if (inv_vec[i] == 1) {
            link_list[i].set_theta(-joint_angs[i]);
        }
        else {
            link_list[i].set_theta(joint_angs[i]);
        }

    }

    //Update the offsets
    update_link_offsets();

    //Recalculate new position
    update_pos_orient();
}

//Updates the poisiton vector and orient based on the transformation matrix
void RobotDH::update_pos_orient(){
    //Calculate the transform matrix
    calc_transform();
    
    //update the position vector
    pos = { trans_mat(0, 3), trans_mat(1, 3), trans_mat(2, 3) };

    //Update the orientation matrix
    orient_mat << trans_mat(0,0), trans_mat(0,1), trans_mat(0,2),
                  trans_mat(1,0), trans_mat(1,1), trans_mat(1,2),
                  trans_mat(2,0), trans_mat(2,1), trans_mat(2,2);


}

//Gets the posstion in XYZ coords
std::vector<float> RobotDH::get_pos(){
    return pos;
}

//Gets the orientation matrix
Matrix3f RobotDH::get_orient(){
    return orient_mat;
}


//Updates the link offsets (primarily for coupled joints)
void RobotDH::update_link_offsets(){        

    coupled_offsets.clear();

    //Check if offsets are coupled with other joints
    for(std::string c : offset_config){

        //Positive coupling
        if (c.substr(0, 1).compare("p") == 0){    

            coupled_offsets.push_back(link_list.at(std::stoi(c.substr(1, c.size()))).get_theta());          

        }
        //Negative Coupling
        else if (c.substr(0, 1).compare("m") == 0){
            coupled_offsets.push_back(-link_list.at(std::stoi(c.substr(1, c.size()))).get_theta());
            
        }
        else{
            coupled_offsets.push_back(0);
        }

    }


    //For the coupled joint offsets - update the link offsets
    for(int i = 0; i < n_of_links; i++){

        link_list[i].set_off(coupled_offsets[i]);

    }
}

//Adds a new link to the robot
void RobotDH::add_link(DHLink new_link, std::string config){

    link_list.push_back(new_link);

    offset_config.push_back(config);

    //Update the coupled offsets
    update_link_offsets();

    //Update the position and orientation
    update_pos_orient();
}

//Removes a link for a given index
//Be very careful when removing links that are coupled - will produce unexpected behaviour
void RobotDH::remove_link(int link_n){

    link_list.erase(link_list.begin() + link_n);
    offset_config.erase(offset_config.begin() + link_n);

}


//Calculates the ZYX euler rotation for the TCP
std::vector<float> RobotDH::calc_ZYX_euler(){

    float Z = atan2(orient_mat(1,0), orient_mat(0,0));
    float Y = asin(-orient_mat(2, 0));
    float X = atan2(orient_mat(2,1), orient_mat(2,2));

    std::vector<float> euler = {Z, Y, X};

    return euler;

}


//Calculates the jacobian for the current TCP pos - only works for all rotational joints atm
//Jacobian calculated based on Robot Modelling - M.Spong 
MatrixXf RobotDH::get_jacobian(){

        MatrixXf J = MatrixXf::Zero(6, link_list.size());
        Matrix4f curr_trans; 

        std::vector<float> z_i_minus_one(3);
        std::vector<float> o_i_minus_one(3);
        std::vector<float> o_diff(3);

        //Get the origin of the final reference frame
        std::vector<float> o_n = get_pos();

        //Calculate each column of the jacobian
        for(int i = 0; i < link_list.size(); i++){

            
            //Calculate the current transformation matrix (joint by joint)
            if (i == 0){
                curr_trans = link_list[i].get_hg();
            }

            else{
                curr_trans = curr_trans * link_list[i].get_hg();
            }


            z_i_minus_one = {curr_trans(0, 2), curr_trans(1, 2), curr_trans(2,2)};

            o_i_minus_one = {curr_trans(0, 3), curr_trans(1, 3), curr_trans(2, 3)};

            for(int j = 0; j < o_diff.size(); j++){
                o_diff[i] = o_n[i] - o_i_minus_one[i];
            }


            //The ith J-column is a column vector of <z_i x (o_n - o_i-1), z_i-i> (6x1) for a prismatic joint

            J(0, i) = (z_i_minus_one[1]*o_diff[2]) - (z_i_minus_one[2]*o_diff[1]);
            J(1, i) = -((z_i_minus_one[0]*o_diff[2])-(z_i_minus_one[2]*o_diff[0]));
            J(2, i) = ((z_i_minus_one[0]*o_diff[1])-(z_i_minus_one[1]*o_diff[0]));
            J(3, i) = z_i_minus_one[0];
            J(4, i) = z_i_minus_one[1];
            J(5, i) = z_i_minus_one[2];

        }

        return J;

}

//Returns whether this object has an associated  text model 
bool RobotDH::is_valid(){

    return valid_model;
}








