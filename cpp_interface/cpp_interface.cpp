#include "cpp_interface.h"
#include "iostream"

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

Cpp_interface::Cpp_interface()
{

}

//! All angles input and output unit: radians.

//! *ptr is a c style dynamic structure.
//! machines is the nr of machines results in nr of kinematic chains.
//! joints[x] is a joint array for 20 machines, each machine has a x nr of joints.
MACHINE Cpp_interface::forward_kinematic(MACHINE *ptr, unsigned int machines, int joints[20], bool debug){

    if(debug){
        std::cout<<""<<std::endl;
        std::cout<<"debug print from kinematic cpp interface"<<std::endl;
    }

    for(unsigned int i=0; i<machines; i++){

        //! For every machine we need :
        KDL::Chain KDLChain;
        KDL::JntArray KDLJointInit;
        KDL::JntArray KDLJointCur;
        KDL::JntArray KDLJointMin;
        KDL::JntArray KDLJointMax;
        KDL::Frame KDLCart;

        for(int j=0; j<joints[i]; j++){
            if(debug){
                std::cout<<"joint:"<<j<<"position:"<<ptr[i].joint[j].current<<std::endl;
                std::cout<<"joint:"<<j<<"axis endpoint x:"<<ptr[i].joint[j].axis_endpoint.x<<std::endl;
                std::cout<<"joint:"<<j<<"axis endpoint x:"<<ptr[i].joint[j].axis_endpoint.y<<std::endl;
                std::cout<<"joint:"<<j<<"axis endpoint x:"<<ptr[i].joint[j].axis_endpoint.z<<std::endl;
            }
            //! Setup the chain :
            std::string name="M"+std::to_string(i)+"-J"+std::to_string(j);

            //! Enum joint type.
            //! # rotationtype : rot_x = 0, rot_y = 1, rot_z = 2, trans_x = 3, trans_y = 4, trans_z = 5, rot_axis = 6, trans_axis =7, none = 8
            KDL::Joint::JointType type;

            if((ptr+i)->joint[j].rotationtype==0){
                type=KDL::Joint::RotX;
            }
            if((ptr+i)->joint[j].rotationtype==1){
                type=KDL::Joint::RotY;
            }
            if((ptr+i)->joint[j].rotationtype==2){
                type=KDL::Joint::RotZ;
            }
            if((ptr+i)->joint[j].rotationtype==3){
                type=KDL::Joint::TransX;
            }
            if((ptr+i)->joint[j].rotationtype==4){
                type=KDL::Joint::TransY;
            }
            if((ptr+i)->joint[j].rotationtype==5){
                type=KDL::Joint::TransZ;
            }
            if((ptr+i)->joint[j].rotationtype==6){
                type=KDL::Joint::RotAxis;
            }
            if((ptr+i)->joint[j].rotationtype==7){
                type=KDL::Joint::TransAxis;
            }
            if((ptr+i)->joint[j].rotationtype==8){
                type=KDL::Joint::None;
            }
            KDLChain.addSegment(KDL::Segment(name,KDL::Joint(type), KDL::Frame(KDL::Vector((ptr+i)->joint[j].axis_endpoint.x, (ptr+i)->joint[j].axis_endpoint.y, (ptr+i)->joint[j].axis_endpoint.z))));
            if(debug){
                std::cout<<"joint-setup x:"<<(ptr+i)->joint[j].axis_endpoint.x<<" y:"<<(ptr+i)->joint[j].axis_endpoint.y<<" z:"<<(ptr+i)->joint[j].axis_endpoint.z<<std::endl;
                std::cout<<"joint-rotation type:"<<(ptr+i)->joint[j].rotationtype<<std::endl;
            }
        }

        KDLJointMin.resize(KDLChain.getNrOfSegments());
        KDLJointMax.resize(KDLChain.getNrOfSegments());
        KDLJointCur.resize(KDLChain.getNrOfSegments());
        KDLJointInit.resize(KDLChain.getNrOfSegments());

        for(int j=0; j<joints[i]; j++){
            KDLJointInit(j)= (ptr+i)->joint[j].init; //! Unit: radians
            KDLJointCur(j)= (ptr+i)->joint[j].current;
            KDLJointMin(j)= (ptr+i)->joint[j].min;
            KDLJointMax(j)= (ptr+i)->joint[j].max;
        }

        //! Perform a forward kinematic calculation (fk).
        KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(KDLChain);
        int status = fksolver.JntToCart(KDLJointCur,KDLCart,-1);

        if(status==0 || status>0){
            (ptr+i)->cart.x=KDLCart.p.x();
            (ptr+i)->cart.y=KDLCart.p.y();
            (ptr+i)->cart.z=KDLCart.p.z();
            KDLCart.M.GetEulerZYX((ptr+i)->euler.z,(ptr+i)->euler.y,(ptr+i)->euler.x);
            (ptr+i)->error=0;

            if(debug){
                std::cout<<"cartx:"<<(ptr+i)->cart.x<<std::endl;
                std::cout<<"carty:"<<(ptr+i)->cart.y<<std::endl;
                std::cout<<"cartz:"<<(ptr+i)->cart.z<<std::endl;
                std::cout<<"eulerx:"<<(ptr+i)->euler.x<<std::endl;
                std::cout<<"eulery:"<<(ptr+i)->euler.y<<std::endl;
                std::cout<<"eulerz:"<<(ptr+i)->euler.z<<std::endl;
            }
        } else {
            if(debug){
                std::cout<<fksolver.getError()<<std::endl;
                std::cout<<"fk error"<<std::endl;
            }
            (ptr+i)->error=1;
        }

    }
    return *ptr;
}

extern "C" MACHINE forward_kinematic(MACHINE *ptr, unsigned int machines, int joints[20], bool debug){
    MACHINE result_ptr=Cpp_interface().forward_kinematic(ptr,machines,joints,debug);
    return result_ptr;
}

//! *ptr is a c style dynamic structure.
//! machines is the nr of machines results in nr of kinematic chains.
//! joints[x] is a joint array for 20 machines, each machine has a x nr of joints.
MACHINE Cpp_interface::inverse_kinematic(MACHINE *ptr, unsigned int machines, int joints[20], bool debug){

    if(debug){
        std::cout<<""<<std::endl;
        std::cout<<"debug print from kinematic cpp interface"<<std::endl;
    }

    for(unsigned int i=0; i<machines; i++){

        //! For every machine we need :
        KDL::Chain KDLChain;
        KDL::JntArray KDLJointInit;
        KDL::JntArray KDLJointCur;
        KDL::JntArray KDLJointMin;
        KDL::JntArray KDLJointMax;
        KDL::Frame KDLCart;

        for(int j=0; j<joints[i]; j++){
            if(debug){
                std::cout<<"joint:"<<j<<"position:"<<ptr[i].joint[j].current<<std::endl;
                std::cout<<"joint:"<<j<<"axis endpoint x:"<<ptr[i].joint[j].axis_endpoint.x<<std::endl;
                std::cout<<"joint:"<<j<<"axis endpoint x:"<<ptr[i].joint[j].axis_endpoint.y<<std::endl;
                std::cout<<"joint:"<<j<<"axis endpoint x:"<<ptr[i].joint[j].axis_endpoint.z<<std::endl;
            }
            //! Setup the chain :
            std::string name="M"+std::to_string(i)+"-J"+std::to_string(j);


            //! Enum joint type.
            //! # rotationtype : rot_x = 0, rot_y = 1, rot_z = 2, trans_x = 3, trans_y = 4, trans_z = 5, rot_axis = 6, trans_axis =7, none = 8
            KDL::Joint::JointType type;
            if((ptr+i)->joint[j].rotationtype==0){
                type=KDL::Joint::RotX;
            }
            if((ptr+i)->joint[j].rotationtype==1){
                type=KDL::Joint::RotY;
            }
            if((ptr+i)->joint[j].rotationtype==2){
                type=KDL::Joint::RotZ;
            }
            if((ptr+i)->joint[j].rotationtype==3){
                type=KDL::Joint::TransX;
            }
            if((ptr+i)->joint[j].rotationtype==4){
                type=KDL::Joint::TransY;
            }
            if((ptr+i)->joint[j].rotationtype==5){
                type=KDL::Joint::TransZ;
            }
            if((ptr+i)->joint[j].rotationtype==6){
                type=KDL::Joint::RotAxis;
            }
            if((ptr+i)->joint[j].rotationtype==7){
                type=KDL::Joint::TransAxis;
            }
            if((ptr+i)->joint[j].rotationtype==8){
                type=KDL::Joint::None;
            }
            KDLChain.addSegment(KDL::Segment(name,KDL::Joint(type), KDL::Frame(KDL::Vector((ptr+i)->joint[j].axis_endpoint.x, (ptr+i)->joint[j].axis_endpoint.y, (ptr+i)->joint[j].axis_endpoint.z))));
            if(debug){
                std::cout<<"joint-setup x:"<<(ptr+i)->joint[j].axis_endpoint.x<<" y:"<<(ptr+i)->joint[j].axis_endpoint.y<<" z:"<<(ptr+i)->joint[j].axis_endpoint.z<<std::endl;
                std::cout<<"joint-rotation type:"<<(ptr+i)->joint[j].rotationtype<<std::endl;
            }
        }

        KDLJointMin.resize(KDLChain.getNrOfSegments());
        KDLJointMax.resize(KDLChain.getNrOfSegments());
        KDLJointCur.resize(KDLChain.getNrOfSegments());
        KDLJointInit.resize(KDLChain.getNrOfSegments());

        for(int j=0; j<joints[i]; j++){
            KDLJointInit(j)= ptr[i].joint[j].init;
            KDLJointCur(j)= ptr[i].joint[j].current;
            KDLJointMin(j)= ptr[i].joint[j].min;
            KDLJointMax(j)= ptr[i].joint[j].max;
        }

        KDLCart.p.x(ptr[i].cart.x);
        KDLCart.p.y(ptr[i].cart.y);
        KDLCart.p.z(ptr[i].cart.z);

        // Testparams for kuka robot.
        // KDLCart.p.x(500);
        // KDLCart.p.y(0);
        // KDLCart.p.z(500);
        // KDLCart.M.EulerZYX(d.Eulerz,d.Eulery,d.Eulerx);

        KDLCart.M.DoRotZ(ptr[i].euler.z-KDLCart.M.GetRot().z());
        KDLCart.M.DoRotY(ptr[i].euler.y-KDLCart.M.GetRot().y());
        KDLCart.M.DoRotX(ptr[i].euler.x-KDLCart.M.GetRot().x());

        KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(KDLChain);

        KDL::ChainIkSolverVel_pinv iksolverv(KDLChain); //Inverse velocity solver needed for IK
        if(ptr[i].kinematic_iterations==0){ptr[i].kinematic_iterations=100;}
        KDL::ChainIkSolverPos_NR_JL iksolver(KDLChain, KDLJointMin, KDLJointMax, fksolver, iksolverv, ptr[i].kinematic_iterations, 1e-6); //max 100 iterations, stop at accuracy 1e-6
        KDL::JntArray JntResult(KDLChain.getNrOfJoints());

        int status=0;
        if(ptr[i].perform_ik_init){ // Perform Ik from Init, we can add a hal input pin for this.
            status = iksolver.CartToJnt(KDLJointInit, KDLCart, JntResult);
        }
        if(ptr[i].perform_ik_current){
            status = iksolver.CartToJnt(KDLJointCur, KDLCart, JntResult);
        }
        if(status>=0){
            for(int j=0; j<joints[i]; j++){
                ptr[i].joint[j].current=JntResult(j);
            }
            ptr[i].error=0;
        } else {
            if(debug){
                std::cout<<iksolver.getError()<<std::endl;
                std::cout<<"ik error"<<std::endl;
            }
            ptr[i].error=1;
        }
    }
    return *ptr;
}

extern "C" MACHINE inverse_kinematic(MACHINE *ptr, unsigned int machines, int joints[20], bool debug){
    MACHINE result_ptr=Cpp_interface().inverse_kinematic(ptr,machines,joints,debug);
    return result_ptr;
}














