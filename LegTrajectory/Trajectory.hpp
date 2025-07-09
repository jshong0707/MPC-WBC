#pragma once

#include "globals.hpp"
#include "F_Kinematics.hpp"
#include "B_Kinematics.hpp"
#include "Body.hpp"
#include "BezierCurve.hpp"
#include "robot_parameter.hpp"

class BezierCurve;
class Body;
class F_Kinematics;

class Trajectory
{
private:
    BezierCurve *Bezier;
    Body &B_;
    robot_parameter &pino;
    
    VectorXd pos_ref;
    
    int N_task;

    /* Gait */
        double t_norm = 0;
        vector<double> gait_phase_delay;
        MatrixXd Bz_points;
        double swing_ratio; // swing phase 비율
        double T_swing;
        double T_stance;
        double T_total;
        Vector3d pdot_ref = Vector3d::Zero();
        Vector3d pdot = Vector3d::Zero();
        
        double stride;
        vector<bool> is_contact = {true, true, true, true};
        vector<bool> old_contact = {true, true, true, true};
        
        Vector3d stance_end_pos[4];
        VectorXd x_ref;
        VectorXd x0;
        Vector3d CP;
        double w;
        
public:
    Trajectory(robot_parameter &pino, Body &B_);
    ~Trajectory();

    VectorXd custom_leg_traj(double t);
    VectorXd Hold();
    VectorXd swing_traj(double t);
    VectorXd Traj(double t);
    Vector3d get_foot_pos(int Leg_num);
    vector<bool> FSM(){return is_contact;}

    
};

