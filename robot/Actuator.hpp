#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "globals.hpp"


class filter;

class Actuator
{

private:
    filter *F;

    int ACT_num;
    double d_cutoff = 70;
    double q = 0;
    double qd_tustin = 0;
    double qdd_tustin = 0;
    
    double q_old = 0;
    double qd_tustin_old = 0;
    double qdd_tustin_old = 0;    
    


public:
    Actuator(int n);
    ~Actuator();
    Vector3d Receive_data(const mjModel* m, mjData* d);

};

#endif