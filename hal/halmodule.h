#ifndef HALMODULE_H
#define HALMODULE_H

#include "rtapi.h"
#ifdef RTAPI
#include "rtapi_app.h"
#endif
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "hal.h"
#include "rtapi_math64.h"
#include "rtapi_math.h"
#include <stdio.h>
#include <stdlib.h>

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
    //! //! Kdl kinematic setup input values for a axis or joint.
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

extern struct MACHINE forward_kinematic(struct MACHINE *ptr, unsigned int machines, int joints[20], bool debug);
extern struct MACHINE inverse_kinematic(struct MACHINE *ptr, unsigned int machines, int joints[20], bool debug);

// Example function:
void myfirstvoid(float a, float b, float *result);

void myfirstvoid(float a, float b, float *result){
    *result=a+b;
}

#endif // HALMODULE_H































