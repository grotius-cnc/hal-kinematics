#ifndef NEXT_H
#define NEXT_H

#include "next_global.h"
#include <string>

#ifndef ULAPI
#define ULAPI
#endif

#define SUCCESS 1
#undef Success //https://eigen.tuxfamily.org/bz/show_bug.cgi?id=253

#include <chainiksolverpos_lma.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolvervel_pinv.hpp>
#include <chainiksolverpos_nr_jl.hpp>

#include <iostream>
#include <chrono>
#include <thread>

struct point {
    double x,y,z;
};

struct data {

    // Joint 0
    double J0_x;
    double J0_y;
    double J0_z;

    double J0_init;
    double J0_cur;
    double J0_min;
    double J0_max;

    // Joint 1
    double J1_x;
    double J1_y;
    double J1_z;

    double J1_init;
    double J1_cur;
    double J1_min;
    double J1_max;

    // Joint 2
    double J2_x;
    double J2_y;
    double J2_z;

    double J2_init;
    double J2_cur;
    double J2_min;
    double J2_max;

    // Joint 3
    double J3_x;
    double J3_y;
    double J3_z;

    double J3_init;
    double J3_cur;
    double J3_min;
    double J3_max;

    // Joint 4
    double J4_x;
    double J4_y;
    double J4_z;

    double J4_init;
    double J4_cur;
    double J4_min;
    double J4_max;

    // Joint 4
    double J5_x;
    double J5_y;
    double J5_z;

    double J5_init;
    double J5_cur;
    double J5_min;
    double J5_max;

    // joints
    double J0,J1,J2,J3,J4,J5;
    // cartesian
    double Cartx,Carty,Cartz;
    // eulers
    double Eulerx,Eulery,Eulerz;

    int Fk_mode;
    int Ik_mode;
};

class NEXT_EXPORT Next{
public:

    Next();

    data init_wrapper(data d);
    data init(data d);

    data fk_wrapper(data d);
    data fk(data d);

    data ik_wrapper(data d);
    data ik(data d);

    //! Kdl data storage:
    KDL::Chain KDLChain;
    KDL::JntArray KDLJointInit;
    KDL::JntArray KDLJointCur;
    KDL::JntArray KDLJointMin;
    KDL::JntArray KDLJointMax;
    KDL::Frame cart,cartzero;

};

#endif // NEXT_H
