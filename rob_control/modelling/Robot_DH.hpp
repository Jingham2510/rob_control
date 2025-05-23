#include "DH_Link.hpp"
#include <math.h>
#include <vector>

#include <string>
#include <Eigen/Dense>
#include<iostream>
#include<fstream>

using Eigen::MatrixXf;
using Eigen::Matrix3f;

#ifndef Robot_DH
#define Robot_DH

//Class that defines the DH representation of a robot
class RobotDH{
    public:
                
       //Default constructor
        RobotDH();

        //Constructor
        RobotDH(std::vector<DHLink>, std::vector<std::string>, std::vector<int>);

        //Alternative constructor when the model is known
        RobotDH(std::string);


        //Move assignment -- overwrite everything
        RobotDH& operator=(const RobotDH& other) noexcept{


            //Copy all the stuff across without calculating it again
            this->link_list = other.link_list;
            this->offset_config = other.offset_config;
            this->inv_vec = other.inv_vec;

            this->coupled_offsets = other.coupled_offsets;
            this->trans_mat = other.trans_mat;
            this->pos = other.pos;
            this->orient_mat = other.orient_mat;
            this->n_of_links = other.n_of_links;

            this->valid_model = other.valid_model;
            return *this;
        }


        //Add a link to the link list
        void add_link(DHLink, std::string);
        //Remove a link from the link list
        void remove_link(int);

        //Gets the transform matrix
        Matrix4f get_trans();

        //Gets the position vector
        std::vector<float> get_pos();

        //Gets the orientation matrix
        Matrix3f get_orient();

        //Updates thetas
        void update_joints(std::vector<float>);

        //Calculates the euler ZYX orient
        std::vector<float> calc_ZYX_euler();

        //Calculates the jacobian
        MatrixXf get_jacobian();

        //Gets whether the model is valid
        bool is_valid();

        

    private:

        const std::vector<std::string> VALID_MODELS = {"IRB 6400R-200/2.8"};
        const std::vector<std::string> MODEL_FILENAMES = {"IRB_6400.txt"};

        //Represnts whether this object is working with a valid model
        bool valid_model;

        //Contains the links representing the robot
        std::vector<DHLink> link_list;
        //couplde offsets (when the offsets are tied to other joints)
        std::vector<std::string> offset_config;
        std::vector<float> coupled_offsets;

        //Vector which holds whether angles have to be inverted
        // 1 = invert
        std::vector<int> inv_vec;

        //Transformation matrix
        Matrix4f trans_mat;

        //Position vector
        std::vector<float> pos;
        //Orientation matrix
        Matrix3f orient_mat;

        //Number of links
        int n_of_links;

        //Check that every joint is within it's limits - this is implemented per robot as some have couplde angles etc
        void check_joint_lims();

        //Calculates the transformation matrix
        void calc_transform();

        //Calculates the position and orient matrix
        void update_pos_orient();

        //Updates the link offsets
        void update_link_offsets();

        //Calculates every joints position
        std::vector<std::vector<float>> calc_joint_poses();

        
        
};





#endif