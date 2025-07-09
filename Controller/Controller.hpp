#pragma once

#include "globals.hpp"
#include "filter.hpp"
#include "eigen-master/Eigen/Core"
#include "eigen-master/Eigen/Dense"
#include "FSM.hpp"
#include "Body.hpp"
#include "MPC.hpp"

using namespace Eigen;
using namespace std;

class filter;
class F_Kinematics;
class B_Kinematics;

class Controller
{
private:
    // filtertool filtere;
    filter* Filter;
    
    double cutoff_freq = 70;
    double tau = 0.01;

    Vector3d P_term[4];
    Vector3d P_term_old[4];
    Vector3d I_term[4];
    Vector3d I_term_old[4];
    Vector3d D_term[4];
    Vector3d D_term_old[4];
    Vector3d PID_output[4];

    Vector3d FL_ctrl_input = Vector3d::Zero();
    Vector3d FR_ctrl_input = Vector3d::Zero();
    Vector3d RL_ctrl_input = Vector3d::Zero();
    Vector3d RR_ctrl_input = Vector3d::Zero();

    Vector3d FL_Joint_input = Vector3d::Zero();
    Vector3d FR_Joint_input = Vector3d::Zero();
    Vector3d RL_Joint_input = Vector3d::Zero();
    Vector3d RR_Joint_input = Vector3d::Zero();


    // Gain //
    Vector3d KP[4];
    Vector3d KI[4];
    Vector3d KD[4];
    

    
    Vector4d pos_KP;
    Vector4d pos_KD;



    bool FB_mode = 0; // 0 pos 1 vel
    bool RWDOB_flag = true;
    bool Orientation_DOB_flag = true;
    bool* Contact;


public:



    Controller();  
    ~Controller();
   
    Vector3d FB_controller(Vector3d error, Vector3d error_old, int Leg_num);
    void Leg_controller(F_Kinematics &K_FL, F_Kinematics &K_FR, B_Kinematics &K_RL, B_Kinematics &K_RR);


    
    
    bool get_FB_mode() {return FB_mode;}
    
    Vector3d get_FL_M_input() {return FL_Joint_input;}
    Vector3d get_FR_M_input() {return FR_Joint_input;}
    Vector3d get_RL_M_input() {return RL_Joint_input;}
    Vector3d get_RR_M_input() {return RR_Joint_input;}
    
    
};

