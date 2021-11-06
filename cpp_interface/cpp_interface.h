#ifndef CPP_INTERFACE_H
#define CPP_INTERFACE_H

//! Author : Skynet
//! Optimalized kinematic library implementation
//!  - Dynamic kinematic setup.
//!  - Setup multiple independent kinematic chains, strains.

#include "cpp_interface_global.h"
#include <string>
#include <iostream>

#ifndef ULAPI
#define ULAPI
#endif

#define SUCCESS 1
#undef Success //https://eigen.tuxfamily.org/bz/show_bug.cgi?id=253

#include <chainiksolverpos_lma.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolvervel_pinv.hpp>
#include <chainiksolverpos_nr_jl.hpp>

#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

//! Represent a 3d point in xyz space.
struct POINT {
    double x,y,z;
};

//! Kdl euler angles. We do not use Yaw,Roll and Pitch values to avoid confusing.
struct EULER {
    double x,y,z;
};

//! Every machine can have multiple joints to form a kinematic structure or chain.
struct JOINT {
    //! Joint type, rotation or translation.
    //! # rotationtype : rot_x = 0, rot_y = 1, rot_z = 2, trans_x = 3, trans_y = 4, trans_z = 5, rot_axis = 6, trans_axis =7, none = 8
    double rotationtype;
    //! Kinematic chain, coordinates end of axis.
    struct POINT axis_endpoint;
    //! Joint values in degrees
    double init;
    double current;
    double min;
    double max;
};

struct MACHINE {
  //! Multiple joints for a machine are possible.
  struct JOINT *joint;
  //! Cartesian result from inverse kinematics.
  struct POINT cart;
  //! Euler result from inverse kinematics.
  struct EULER euler;
  //! Kinematic ammount of calculations to find optimal kinematic result ~100
  int kinematic_iterations;
  int kinematic_error;
  //! Wich type of calculation is requested.
  bool perform_fk;
  bool perform_ik_current;
  bool perform_ik_init;
  bool error;
};
//! Pointer that holds the machine data of multiple machines.
struct MACHINE *ptr;

class CPP_INTERFACE_EXPORT Cpp_interface{
public:
    Cpp_interface();
    MACHINE forward_kinematic(MACHINE *ptr, unsigned int machines, int joints[20], bool debug);
    MACHINE inverse_kinematic(MACHINE *ptr, unsigned int machines, int joints[20], bool debug);
};

#endif // CPP_INTERFACE_H
