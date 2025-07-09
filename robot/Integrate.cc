#include "Integrate.hpp"


Integrate::Integrate(robot_parameter &pino, Trajectory &Traj, Body &B, MPC &M, Controller &C, FSM &FSM_)
:pino(pino), Traj(Traj), B(B), M(M), C(C), FSM_(FSM_)
{
    leg_pos_ref = VectorXd::Zero(12);
    leg_pos = VectorXd::Zero(12);
    
    mpc_dt = M.get_dt();
    opt_u.resize(12); opt_u.setZero();
}

Integrate::~Integrate()
{
}

void Integrate::sensor_measure(const mjModel* m, mjData* d)
{
    t = d->time;

    for(int i = 0; i < 19; i++) q[i] = d->qpos[i];
    for(int i = 0; i < 18; i++) qd[i] = d->qvel[i];
    pino.robot_param(q, qd);

    B.sensor_measure(m, d);
    M.foot_vector(m, d);
    
}

void Integrate::Leg_controller()
{
    /* State Machine */
    is_contact = FSM_.contactschedule();

    /* FeedBack Controller */
    for(int leg = 0; leg < 4; leg++)
    {
        if(is_contact[leg] == false)
            FB_input[leg] = C.FB_controller(pos_err[leg], pos_err_old[leg], leg);
        else
            FB_input[leg] = Vector3d::Zero();
    }

    /* MPC */
    if(t >= optimization_t)
    {
        M.Dynamics();
        M.SolveQP();

        opt_u = M.get_opt_u();
        
        if(t < 0.00000001)
            opt_u = VectorXd::Zero(12);

        for(int i = 0; i < 4; i ++)
        {
            opt_u[3*i] = - opt_u[3*i];
            opt_u[3*i + 1] = - opt_u[3*i + 1];
            opt_GRF[i] = B.get_R().transpose() * opt_u.segment(3*i,3);
        }

        // cout << "///////////////////////////////////////////////////////////\n" << endl;
        // cout << is_contact << endl;
        // cout << opt_GRF[0][0] << "   " << opt_GRF[1][0] << "   " << opt_GRF[2][0] << "   " << opt_GRF[3][0] << endl; 
        // cout << opt_GRF[0][1] << "   " << opt_GRF[1][1] << "   " << opt_GRF[2][1] << "   " << opt_GRF[3][1] << endl; 
        // cout << opt_GRF[0][2] << "   " << opt_GRF[1][2] << "   " << opt_GRF[2][2] << "   " << opt_GRF[3][2] << endl; 
        
        optimization_t += mpc_dt;
    }

    /* Joint Input */

        F_Joint_input[0] = pino.get_Jacb(0).transpose() * (opt_GRF[0] + FB_input[0]);
        F_Joint_input[1] = pino.get_Jacb(1).transpose() * (opt_GRF[1] + FB_input[1]);
        B_Joint_input[0] = pino.get_Jacb(2).transpose() * (opt_GRF[2] + FB_input[2]);
        B_Joint_input[1] = pino.get_Jacb(3).transpose() * (opt_GRF[3] + FB_input[3]);


}


void Integrate::get_error(double t)
{
    leg_pos_ref = Traj.Traj(t);
    
    // cout << leg_pos_ref << endl;

    for(int i = 0; i < 4; i ++)
        pos_err_old[i] = pos_err[i];

    pos_err[0] = leg_pos_ref.segment(0,3) - pino.get_leg_pos(0);
    pos_err[1] = leg_pos_ref.segment(3,3) - pino.get_leg_pos(1);
    pos_err[2] = leg_pos_ref.segment(6,3) - pino.get_leg_pos(2);
    pos_err[3] = leg_pos_ref.segment(9,3) - pino.get_leg_pos(3);

}

void Integrate::Data_log()
{
    leg_pos << pino.get_leg_pos(0), pino.get_leg_pos(1), pino.get_leg_pos(2), pino.get_leg_pos(3);

}   
