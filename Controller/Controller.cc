#include "Controller.hpp"

Controller::Controller()
{

    cutoff_freq = 70;

    
    for(int i = 0; i < 4; i++)
    {
        P_term[i] = Vector3d::Zero();
        I_term[i] = Vector3d::Zero();
        D_term[i] = Vector3d::Zero();
        P_term_old[i] = Vector3d::Zero();
        I_term_old[i] = Vector3d::Zero();
        D_term_old[i] = Vector3d::Zero();
    }

    pos_KP << 1000, 1000, 1000, 1000;  // FL, FR, RL, RR
    pos_KD << 20, 20, 20, 20;

    
// Gain set //
    for(int i = 0; i < 4; i++)
    {
        KP[i][0] = pos_KP[i];          KI[i][0] = 0;              KD[i][0] = pos_KD[i];
        KP[i][1] = pos_KP[i];          KI[i][1] = 0;              KD[i][1] = pos_KD[i];
        KP[i][2] = pos_KP[i];          KI[i][2] = 0;              KD[i][2] = pos_KD[i];
    }

}

Controller::~Controller()
{

}

void Controller::Leg_controller(F_Kinematics &K_FL, F_Kinematics &K_FR, B_Kinematics &K_RL, B_Kinematics &K_RR)
{

    FL_Joint_input =  K_FL.get_Jacb().transpose() * FL_ctrl_input;
    FR_Joint_input =  K_FR.get_Jacb().transpose() * FR_ctrl_input;
    RL_Joint_input =  K_RL.get_Jacb().transpose() * RL_ctrl_input;
    RR_Joint_input =  K_RR.get_Jacb().transpose() * RR_ctrl_input;

    
}

Vector3d Controller::FB_controller(Vector3d error, Vector3d error_old, int Leg_num)
{

    tau = 1 / (2 * M_PI * cutoff_freq);

    P_term[Leg_num] = KP[Leg_num].cwiseProduct(error);

    I_term[Leg_num] = KI[Leg_num].cwiseProduct(Ts / 2 * (error + error_old)) + I_term_old[Leg_num];
    
    D_term[Leg_num] = 2 * KD[Leg_num].cwiseProduct(1 / (2 * tau + Ts) * (error - error_old)) -
                       (Ts - 2 * tau) / (2 * tau + Ts) * D_term_old[Leg_num];

    PID_output[Leg_num] = P_term[Leg_num] + I_term[Leg_num] + D_term[Leg_num];
    

    P_term_old[Leg_num] = P_term[Leg_num];
    I_term_old[Leg_num] = I_term[Leg_num];
    D_term_old[Leg_num] = D_term[Leg_num];

    return PID_output[Leg_num];
}





