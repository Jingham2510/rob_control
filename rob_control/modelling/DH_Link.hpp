
#include<Eigen\Dense>
#include<math.h>

using Eigen::Matrix4f;

#ifndef DH_LINK
#define DH_LINK

//Link that forms part of a DH representation of a robot
//TODO: Add options for prismatic rather than revolutionary joints
class DHLink{
    
    public:
        //Constructor
        DHLink (float,float,float,float, float);
        //Update the joint angle
        void set_theta(float, float);
        //Update the offset
        void set_off(float);
        //Get the current homogeneous transformation matrix
        Matrix4f get_hg();

        //Get theta (joint angle)
        float get_theta();

        

    private:
        //Only accessed once on creation (then just theta)
        float a, alpha, d, theta, base_offset, curr_offset;
        //The homogeneous transformation matrix
        Matrix4f hg_mat;

        //Should only be called when parameters change
        void update_hg();


};
#endif