#include "Body.hpp"

Body::Body(robot_parameter &pino):pino(pino)
{
    N_Task = 1;   // 0: Hold, 1: trot
    x0.resize(12);
    x0.setZero();
    x_ref.resize(12);
    x_ref.setZero();
    
    M = 17;
    I(0,0) = 0.289178;
    I(1,1) = 0.539984;
    I(2,2) = 0.829139;
}

Body::~Body()
{
}

Matrix3d Body:: get_R()
{
    
    Vector3d RPY = {x0[0], x0[1], x0[2]};
    Vector3d RPY2 = pino.get_rpy();
    Matrix3d R2;
    //     Matrix3d RR;
    // // Rotation matrix (Yaw-Pitch-Roll) Body to world
        R << cos(RPY[2]) * cos(RPY[1]), 
            cos(RPY[2]) * sin(RPY[1]) * sin(RPY[0]) - sin(RPY[2]) * cos(RPY[0]), 
            cos(RPY[2]) * sin(RPY[1]) * cos(RPY[0]) + sin(RPY[2]) * sin(RPY[0]),

            sin(RPY[2]) * cos(RPY[1]), 
            sin(RPY[2]) * sin(RPY[1]) * sin(RPY[0]) + cos(RPY[2]) * cos(RPY[0]), 
            sin(RPY[2]) * sin(RPY[1]) * cos(RPY[0]) - cos(RPY[2]) * sin(RPY[0]),

            -sin(RPY[1]), 
            cos(RPY[1]) * sin(RPY[0]), 
            cos(RPY[1]) * cos(RPY[0]);

        R2 << cos(RPY2[2]) * cos(RPY2[1]), 
            cos(RPY2[2]) * sin(RPY2[1]) * sin(RPY2[0]) - sin(RPY2[2]) * cos(RPY2[0]), 
            cos(RPY2[2]) * sin(RPY2[1]) * cos(RPY2[0]) + sin(RPY2[2]) * sin(RPY2[0]),

            sin(RPY2[2]) * cos(RPY2[1]), 
            sin(RPY2[2]) * sin(RPY2[1]) * sin(RPY2[0]) + cos(RPY2[2]) * cos(RPY2[0]), 
            sin(RPY2[2]) * sin(RPY2[1]) * cos(RPY2[0]) - cos(RPY2[2]) * sin(RPY2[0]),

            -sin(RPY2[1]), 
            cos(RPY2[1]) * sin(RPY2[0]), 
            cos(RPY2[1]) * cos(RPY2[0]);

    return R;
}

VectorXd Body:: get_x_ref(double t)
{
    // x_ref << 0,     // roll
    //         0,      // pitch    
    //         0.0*t,      // yaw
    //         0.2 * t,      // x
    //         0,      // y
    //         0.3536, // z
    //         0,      // roll dot
    //         0,      // pitch dot
    //         0.,      // yaw dot
    //         0.2,      // x dot
    //         0,      // y dot
    //         0;      // z dot

    if(t<5)
        x_ref << 0,     // roll
            0,      // pitch    
            0.2*t,      // yaw
            0.2 * t,      // x
            0.2*t,      // y
            0.3536, // z
            0,      // roll dot
            0,      // pitch dot
            0.2,      // yaw dot
            0.2,      // x dot
            0.2,      // y dot
            0;      // z dot
    else
            x_ref << 0,     // roll
            0,      // pitch    
            0.0,      // yaw
            0.3 * t,      // x
            0,      // y
            0.3536, // z
            0,      // roll dot
            0,      // pitch dot
            0.0,      // yaw dot
            0.3,      // x dot
            0,      // y dot
            0;      // z dot

    // // Front jump
    // if(t < 1)
    // {
    //     x_ref[5] = 0.3536 - 0.2536*t;
    //     x_ref[11] = -0.2536;
    // }
    // else if(t < 1.5)
    // {
    //     x_ref[5] = 0.1;
    //     x_ref[11] = 0;        
    // }
    // else
    // {
    //     x_ref[3] = 1*t;
    //     x_ref[9] = 1;

    //     x_ref[5] = 0.1 + 2*t;
    //     x_ref[11] = 2;    
            
    // }
    
    // run
    // if(t < 1)
    // {
    //     x_ref[3] = 0.1*t;
    //     x_ref[9] = 0.1;
    // }
    // else if(t < 2)
    // {
    //     x_ref[3] = 0.2*t + 0.1;
    //     x_ref[9] = 0.2;        
    // }
    // else
    // {
    //     x_ref[3] = 0.25*t + 0.3;
    //     x_ref[9] = 0.25;   
    // }



    return x_ref;
}
VectorXd Body::get_z_ref(double t)
{    
    for(int k = 0; k < horizon; k++ )
        z_ref.segment(k*2*nx, nx) = get_x_ref(t + k*MPC_dt);

    return z_ref;
} 

void Body::init_x_ref_mpc(int n, double sampling_time) {
        horizon = n;
        MPC_dt = sampling_time;
        z_ref.resize(x_ref.size() * horizon * 2);
        z_ref.setZero();
}


void Body::sensor_measure(const mjModel* m, mjData* d)
{


    foot_vector(m, d);
    
    Vector4d quat;
    quat << d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6];
    Vector3d th;
    th = F->quat2rpy(quat);
    
    omega_IMU << d->sensordata[3], d->sensordata[4], d->sensordata[5]; 


    // World Frame
    x0 << th[0], th[1], th[2],               // th
        d->qpos[0], d->qpos[1], d->qpos[2], // p
        omega_IMU[0], omega_IMU[1], omega_IMU[2], // thdot
        d->qvel[0], d->qvel[1], d->qvel[2]; // pdot
    
}
void Body::foot_vector(const mjModel* m, mjData* d) {

    CoM_pos_W << d->subtree_com[0], d->subtree_com[1], d->subtree_com[2];
    for(int i = 0; i < 4; i++)
        r_W[i] << d->site_xpos[3*i+3] - CoM_pos_W[0], d->site_xpos[3*i+4] - CoM_pos_W[1], d->site_xpos[3*i+5] - CoM_pos_W[2];

}
