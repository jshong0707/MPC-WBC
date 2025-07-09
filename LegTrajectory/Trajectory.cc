#include "Trajectory.hpp"

Trajectory::Trajectory(robot_parameter &pino, Body &B_)
:pino(pino), B_(B_)
{
    pos_ref = VectorXd::Zero(12);
    T_swing = 0.3;
    T_stance = 0.3;
    T_total = T_swing + T_stance;
    
    gait_phase_delay = {0.0, 0.5, 0.5, 0.0}; // trot
    swing_ratio = 0.5;

    Bz_points.resize(5,3);
    x_ref.resize(12); x_ref.setZero();
    N_task = B_.get_Task();
    w = sqrt(0.36/9.81);
}

Trajectory::~Trajectory()
{

}

VectorXd Trajectory::Traj(double t)
{
    switch (N_task)
    {
        case 0:
            return Hold(); break;
        case 1: 
            return swing_traj(t); break;
    }

    return VectorXd::Zero(12);
}

VectorXd Trajectory::custom_leg_traj(double t)
{
    // FL
    pos_ref[0] = 0; // + 0.05*sin(t);
    pos_ref[1] = 0;
    pos_ref[2] = 0.3536; // - 0.1*sin(t);

    pos_ref[3] = 0; //+ 0.05*sin(t);
    pos_ref[4] = 0;
    pos_ref[5] = 0.3536; // - 0.1*sin(t);

    pos_ref[6] = -0.1;
    pos_ref[7] = 0;
    pos_ref[8] = 0.3536; // - 0.1*sin(t);

    pos_ref[9] = -0.1;
    pos_ref[10] = 0;
    pos_ref[11] = 0.3536; // - 0.1*sin(t);

    return pos_ref;
}

VectorXd Trajectory::swing_traj(double t)
{
    x_ref = B_.get_x_ref(t);
    x0 = B_.get_x0();

    pdot_ref << x_ref[9], x_ref[10], x_ref[11];
    pdot << x0[9], x0[10], x0[11];
    stride = pdot_ref[0]*T_swing;

    for (int leg = 0; leg < 4; ++leg)
    {
        t_norm = t - T_total * gait_phase_delay[leg]  + T_total;
        double phase = fmod(t_norm / T_total, 1.0); // [0,1) 범위로 정규화
        

        Vector3d pos;
        if (phase < swing_ratio) {
            // Swing phase
            double s = phase / swing_ratio;  // 0~1 정규화된 swing phase
            
            is_contact[leg] = false;
            
            if (old_contact[leg] && !is_contact[leg]) {
            stance_end_pos[leg] = get_foot_pos(leg);
            }
            
 
            // Bz_points << stance_end_pos[leg][0], stance_end_pos[leg][1], stance_end_pos[leg][2],
            //             -0.2*stride/2, 0.0, 0.25,
            //             0.5*stride/2, 0.0, 0.31,   
            //             0.8*stride/2, 0.0, 0.34,   
            //             stride/2, 0.0, 0.36;       
            // + w * CP

            Bz_points << stance_end_pos[leg][0], stance_end_pos[leg][1], stance_end_pos[leg][2],
                        -0.2*stride/2 + w * CP[0], w * CP[1], 0.2,
                        0.5*stride/2 + w * CP[0], w * CP[1], 0.25,   
                        0.8*stride/2 + w * CP[0], w * CP[1], 0.34,   
                        stride/2 + w * CP[0], w * CP[1], 0.36;
                               
            CP = pdot - pdot_ref;
            CP[2] = 0;
            pos = Bezier->getBezierCurve(Bz_points, s);
            // cout << sqrt(0.36/9.81) * (pdot - pdot_ref) << endl;

        } else {
            double s = (phase - swing_ratio) / (1.0 - swing_ratio); // 0~1 정규화된 stance phase
            is_contact[leg] = true;
            // Stance phase
            // pos << -stride*s + stride/2  , 0.0, 0.36;
            pos = get_foot_pos(leg);
        }

        old_contact[leg] = is_contact[leg];
        
        pos_ref.segment<3>(leg * 3) = pos;
    }

    return pos_ref;
}

VectorXd Trajectory::Hold()
{
    is_contact = {true, true, true, true};

    for(int leg = 0; leg < 4; leg++)
    pos_ref.segment<3>(leg * 3) = get_foot_pos(leg); 

    return pos_ref;
}
Vector3d Trajectory::get_foot_pos(int Leg_num)
{

    switch (Leg_num)
    {
        case 0:
            return pino.get_leg_pos(0); break;
        case 1:
            return pino.get_leg_pos(1); break;
        case 2:
            return pino.get_leg_pos(2); break;
        case 3:
            return pino.get_leg_pos(3); break;
    }


    return Vector3d::Zero();
}