#include "Actuator.hpp"
#include "filter.hpp"

Actuator::Actuator(int n)
{
    ACT_num = n;
}

Actuator::~Actuator()
{}

Vector3d Actuator::Receive_data(const mjModel* m, mjData* d)
{
    q = d->qpos[ACT_num + 7];
    
    q_old = q;
    
    qd_tustin = F->tustin_derivative(q, q_old, qd_tustin_old, d_cutoff);
    qdd_tustin = F->tustin_derivative(qd_tustin, qd_tustin_old, qdd_tustin_old, d_cutoff);    

    qd_tustin_old = qd_tustin;
    qdd_tustin_old = qdd_tustin;
    
    Vector3d Joint_state;

    Joint_state << q, qd_tustin, qdd_tustin;

    return Joint_state;
}


