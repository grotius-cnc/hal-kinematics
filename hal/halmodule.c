/// Improved by Skynet 03-11-2021
/// Changed from halcompile cody style to c code style.
#include "rtapi.h"
#ifdef RTAPI
#include "rtapi_app.h"
#endif
#include "rtapi_string.h"
#include "rtapi_errno.h"
#include "hal.h"
#include "rtapi_math64.h"
#include "rtapi_math.h"
#include "halmodule.h"

//! Ammount of machines, ammount of kinematic chains.
int machines;
RTAPI_MP_INT(machines,"");

//! Ammount of joints for a machines.
int joints[20];
RTAPI_MP_ARRAY_INT(joints,20,"");

//! Ammount of calculations to find the best kinematic solution.
int iterations;
RTAPI_MP_INT(iterations,"");

//! Print info to terminal.
int debug;
RTAPI_MP_INT(debug,"");

//! Hal component int id.
static int comp_idx;

typedef struct {
    bool ok;
} skynet_t;
skynet_t *skynet;

//! Hal float pin in-out
typedef struct {
    hal_float_t *Pin;
} float_data_t;
float_data_t *cartx_in,*carty_in,*cartz_in;
float_data_t *cartx_out,*carty_out,*cartz_out;
float_data_t *eulerx_in,*eulery_in,*eulerz_in;
float_data_t *eulerx_out,*eulery_out,*eulerz_out;
float_data_t *joint_in;
float_data_t *joint_out;

//! Hal bool pin in-out
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;
bit_data_t *mode_fk;
bit_data_t *mode_ik_current;
bit_data_t *mode_ik_init;
bit_data_t *mode_error; //! using the name error in hal is not allowed.
bit_data_t *enable;

//! Int pin in-out
typedef struct {
    hal_s32_t *Pin;
} s32_data_t;

//! Unsigned int pin in-out
typedef struct {
    hal_u32_t *Pin;
} u32_data_t; 

//! Char array port in-out
typedef struct {
    hal_port_t *Pin;
} port_data_t;

//! Parameter pin rw float
typedef struct {
    hal_float_t Pin;
} param_float_data_t;
param_float_data_t *joint_init;
param_float_data_t *joint_min;
param_float_data_t *joint_max;
param_float_data_t *joint_setup_x;
param_float_data_t *joint_setup_y;
param_float_data_t *joint_setup_z;
param_float_data_t *joint_type;
param_float_data_t *dummy;

//! Parameter pin rw bool
typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;

bool init_struct=0, init_fk=0;

static void function();
static int setup_pins();
static void write_pins();
static void read_pins();
static void allocate_dynamic_c_struct(bool debug);

int rtapi_app_main(void) {

    comp_idx = hal_init("hal_kinematic");
    if(comp_idx < 0) return comp_idx;

    int r = 0;
    r = hal_export_funct("hal_kinematic", function, &skynet,0,0,comp_idx);
    r+=setup_pins();

    if(r) {
        hal_exit(comp_idx);

        //! Eventually free memory
        free(ptr);
        free(ptr->joint);

    } else {
        hal_ready(comp_idx);
    }

    return r;
}

void rtapi_app_exit(void) {
    hal_exit(comp_idx);

    //! Eventually free memory
    free(ptr);
    free(ptr->joint);
}

static void function(){

    if(!init_struct){
        allocate_dynamic_c_struct(debug);
        init_struct=1;
    }

    //! Read actual hal pin's amd param's and apply data to the ptr.
    //!
    //! The _in pin and _out pin's are seperated.
    //! For kinematic calculations the _in pins are used.
    //! The results of the calculations are sent to the _out pins.
    //!
    //! To start up the procedure first do a forward kinematic "mode-fk". This wil give the cart-xyz positions to _out pins.
    //! This procedure is done by component startup only once.
    //!
    //! Then use these info to sent a inverse-kinematic request "mode-ik" to the component with the _in pins containing the cart-xyz values.

    //! Emergency trigger.
    if(*enable->Pin){
        read_pins();

        //! Initialize component always with a forward kinematics.
        if(!init_fk){
            *ptr=forward_kinematic(ptr, machines, joints, debug);
            write_pins();
            init_fk=1;
        }

        //! Perform a c++ function request depending on hal pin state.
        if(*mode_fk->Pin){
            *ptr=forward_kinematic(ptr, machines, joints, debug);
            write_pins();
        }
        if(*mode_ik_init->Pin || *mode_ik_current->Pin){
            *ptr=inverse_kinematic(ptr, machines, joints, debug);
            write_pins();
        }
    }
}

static void read_pins(){
    int kk=0;
    for(int i=0; i<machines; i++){

        (ptr+i)->cart.x=*cartx_in[i].Pin;
        (ptr+i)->cart.y=*carty_in[i].Pin;
        (ptr+i)->cart.z=*cartz_in[i].Pin;

        (ptr+i)->euler.x=*eulerx_in[i].Pin*toRadians;
        (ptr+i)->euler.y=*eulery_in[i].Pin*toRadians;
        (ptr+i)->euler.z=*eulerz_in[i].Pin*toRadians;

        for(int j=0; j<joints[i]; j++){
            (ptr+i)->joint[j].min=joint_min[kk].Pin*toRadians;
            (ptr+i)->joint[j].max=joint_max[kk].Pin*toRadians;
            (ptr+i)->joint[j].init=joint_init[kk].Pin*toRadians;
            (ptr+i)->joint[j].current=*joint_in[kk].Pin*toRadians;

            //! Enum joint type: rot_x = 0, rot_y = 1, rot_z = 2, trans_x = 3, trans_y = 4, trans_z = 5
            (ptr+i)->joint[j].rotationtype=joint_type[kk].Pin;
            //! Axis endpoint xyz setup
            (ptr+i)->joint[j].axis_endpoint.x=joint_setup_x[kk].Pin;
            (ptr+i)->joint[j].axis_endpoint.y=joint_setup_y[kk].Pin;
            (ptr+i)->joint[j].axis_endpoint.z=joint_setup_z[kk].Pin;

            if(debug){
                rtapi_print_msg(RTAPI_MSG_ERR,"JOINT ID:'%d'\n",j);
                rtapi_print_msg(RTAPI_MSG_ERR,"parameter joint min :'%f'\n",joint_min[kk].Pin);
                rtapi_print_msg(RTAPI_MSG_ERR,"parameter joint max :'%f'\n",joint_max[kk].Pin);
                rtapi_print_msg(RTAPI_MSG_ERR,"parameter joint init :'%f'\n",joint_init[kk].Pin);
                rtapi_print_msg(RTAPI_MSG_ERR,"parameter joint current :'%f'\n",*joint_in[kk].Pin);
                rtapi_print_msg(RTAPI_MSG_ERR,"parameter type :'%f'\n",joint_type[kk].Pin);

                rtapi_print_msg(RTAPI_MSG_ERR,"parameter joint setup x :'%f'\n",joint_setup_x[kk].Pin);
                rtapi_print_msg(RTAPI_MSG_ERR,"parameter joint setup y :'%f'\n",joint_setup_y[kk].Pin);
                rtapi_print_msg(RTAPI_MSG_ERR,"parameter joint setup z :'%f'\n",joint_setup_z[kk].Pin);
                rtapi_print_msg(RTAPI_MSG_ERR,"'\n");
            }


            kk++;
        }

        //! Kinematic mode
        (ptr+i)->perform_ik_current=*mode_ik_current->Pin;
        (ptr+i)->perform_ik_init=*mode_ik_init->Pin;
        (ptr+i)->perform_fk=*mode_fk->Pin;

        //! Component argument.
        (ptr+i)->kinematic_iterations=iterations;
    }
}

static void write_pins(){

    //! Write results to hal pin's.
    int k=0;
    for(int i=0; i<machines; i++){

        if(ptr[i].error==0){
            *mode_error[i].Pin=0;
        } else {
            *mode_error[i].Pin=1;
            rtapi_print_msg(RTAPI_MSG_ERR,"Kinematic error at machine id :'%d'\n",i);}

        *cartx_out[i].Pin=(ptr+i)->cart.x;
        *carty_out[i].Pin=(ptr+i)->cart.y;
        *cartz_out[i].Pin=(ptr+i)->cart.z;

        *eulerx_out[i].Pin=(ptr+i)->euler.x*toDegrees;
        *eulery_out[i].Pin=(ptr+i)->euler.y*toDegrees;
        *eulerz_out[i].Pin=(ptr+i)->euler.z*toDegrees;

        for(int j=0; j<joints[i]; j++){
            *joint_out[k].Pin=(ptr+i)->joint[j].current*toDegrees;
            k++;
        }
    }
}

static void allocate_dynamic_c_struct(bool debug){
    if(debug){
        for(int i=0; i<machines; i++){
            rtapi_print_msg(RTAPI_MSG_ERR,"debug printed from kinematic halmodule.c\n");
            rtapi_print_msg(RTAPI_MSG_ERR,"machine :'%d'\n",i);
            rtapi_print_msg(RTAPI_MSG_ERR,"joints :'%d'\n",joints[i]);
        }
    }

    //! Allocate machine structure
    ptr = (struct MACHINE *)malloc(machines * sizeof(struct MACHINE));
    //! Allocate machine joints
    for(int i=0; i<machines; i++){
        (ptr+i)->joint = (struct JOINT *)malloc(joints[i] * sizeof(struct JOINT));
    }
}

static int setup_pins(){
    int r=0;

    mode_ik_init=(bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("hal_kinematic.mode-ik-init",HAL_IN,&(mode_ik_init->Pin),comp_idx);

    mode_ik_current=(bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("hal_kinematic.mode-ik-current",HAL_IN,&(mode_ik_current->Pin),comp_idx);

    mode_fk=(bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("hal_kinematic.mode-fk",HAL_IN,&(mode_fk->Pin),comp_idx);

    enable=(bit_data_t*)hal_malloc(sizeof(bit_data_t));
    r+=hal_pin_bit_new("hal_kinematic.enable",HAL_IN,&(enable->Pin),comp_idx);

    //! Allocate dynamic memory
    int jointsize=0;
    for(int i=0; i<machines; i++){
        for(int j=0; j<joints[i]; j++){
            jointsize++;
        }
    }
    mode_error=(bit_data_t*)hal_malloc(machines* sizeof(bit_data_t));

    cartx_in=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    cartx_out=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    carty_in=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    carty_out=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    cartz_in=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    cartz_out=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    eulerx_in=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    eulerx_out=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    eulery_in=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    eulery_out=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    eulerz_in=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));
    eulerz_out=(float_data_t*)hal_malloc(machines* sizeof(float_data_t));

    joint_in=(float_data_t*)hal_malloc(jointsize* sizeof(float_data_t));
    joint_out=(float_data_t*)hal_malloc(jointsize* sizeof(float_data_t));
    joint_min=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));
    joint_max=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));
    joint_init=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));
    joint_setup_x=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));
    joint_setup_y=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));
    joint_setup_z=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));
    joint_type=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));

    dummy=(param_float_data_t*)hal_malloc(jointsize* sizeof(param_float_data_t));
    // Parameters:
    int k=0;
    for(int i=0; i<machines; i++){

        char str[100]={};
        strcat(str,"hal_kinematic.");
        char nr1[3];
        sprintf(nr1, "%d", i);
        strcat(str,nr1);
        strcat(str,".");

        char text[100]={};
        strcpy(text,str);
        strcat(text,"cartx-in");
        r+=hal_pin_float_new(text,HAL_IN,&(cartx_in[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"cartx-out");
        r+=hal_pin_float_new(text,HAL_OUT,&(cartx_out[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"carty-in");
        r+=hal_pin_float_new(text,HAL_IN,&(carty_in[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"carty-out");
        r+=hal_pin_float_new(text,HAL_OUT,&(carty_out[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"cartz-in");
        r+=hal_pin_float_new(text,HAL_IN,&(cartz_in[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"cartz-out");
        r+=hal_pin_float_new(text,HAL_OUT,&(cartz_out[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"eulerx-in");
        r+=hal_pin_float_new(text,HAL_IN,&(eulerx_in[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"eulerx-out");
        r+=hal_pin_float_new(text,HAL_OUT,&(eulerx_out[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"eulery-in");
        r+=hal_pin_float_new(text,HAL_IN,&(eulery_in[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"eulery-out");
        r+=hal_pin_float_new(text,HAL_OUT,&(eulery_out[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"eulerz-in");
        r+=hal_pin_float_new(text,HAL_IN,&(eulerz_in[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"eulerz-out");
        r+=hal_pin_float_new(text,HAL_OUT,&(eulerz_out[i].Pin),comp_idx);

        strcpy(text,"");
        strcpy(text,str);
        strcat(text,"kinematic-error");
        r+=hal_pin_bit_new(text,HAL_OUT,&(mode_error[i].Pin),comp_idx);

        for(int j=0; j<joints[i]; j++){

            char str[100]={};
            strcat(str,"hal_kinematic.");
            char nr1[3], nr2[3];
            sprintf(nr1, "%d", i);
            sprintf(nr2, "%d", j);
            strcat(str,nr1);
            strcat(str,".");
            strcat(str,nr2);

            char text[100]={};
            strcpy(text,str);
            strcat(text,".joint-max");
            r+=hal_param_float_new(text,HAL_RW,&(joint_max[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-min");
            r+=hal_param_float_new(text,HAL_RW,&(joint_min[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-init");
            r+=hal_param_float_new(text,HAL_RW,&(joint_init[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-type");
            r+=hal_param_float_new(text,HAL_RW,&(joint_type[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-setup-x");
            r+=hal_param_float_new(text,HAL_RW,&(joint_setup_x[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-setup-y");
            r+=hal_param_float_new(text,HAL_RW,&(joint_setup_y[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-setup-z");
            r+=hal_param_float_new(text,HAL_RW,&(joint_setup_z[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-in");
            r+=hal_pin_float_new(text,HAL_IN,&(joint_in[k].Pin),comp_idx);

            strcpy(text,"");
            strcpy(text,str);
            strcat(text,".joint-out");
            r+=hal_pin_float_new(text,HAL_OUT,&(joint_out[k].Pin),comp_idx);

            k++;
        }
    }
    return r;
}






















