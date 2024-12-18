#include"DH_Link.hpp"
#include <iostream>

DHLink::DHLink(float link_length, float link_twist, float link_offset, float joint_angle, float base_off){

    //Set params
    a = link_length;
    alpha = link_twist;
    d = link_offset;
    theta = joint_angle;
    base_offset = base_off;

    //Calc current offset - not coupled automatically
    curr_offset = base_off;

    update_hg();

}

//Compute the homogeneous transofmration matrix as according to M.Spong
void DHLink::update_hg(){

    hg_mat << cos(theta + curr_offset), -sin(theta + curr_offset)*cos(alpha), sin(theta + curr_offset)*sin(alpha),  a*cos(theta + curr_offset),
              sin(theta + curr_offset), cos(theta + curr_offset)*cos(alpha), -cos(theta + curr_offset)*sin(alpha),  a*sin(theta + curr_offset),
                  0     ,                              sin(alpha)   ,                        cos(alpha)   ,                        d     ,
                  0     ,                                   0        ,                            0        ,                       1     ;
        
    //std::cout << cos(theta + curr_offset) << "\n";
      
    return;
}

//Update the joint angle of the link
void DHLink::set_theta(float new_theta, float theta_off){
    theta = new_theta;
    set_off(theta_off);
    update_hg();
    return;
}


//Provide the homogeneous matrix
Matrix4f DHLink::get_hg(){
    return hg_mat;
}


float DHLink::get_theta(){
    return theta;
}

void DHLink::set_off(float theta_off){
    curr_offset = theta_off + base_offset;

    update_hg();

    return;
}

