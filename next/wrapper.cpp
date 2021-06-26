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

//class NEXT_EXPORT Next
class Next {
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

Next::Next()
{

}

data Next::init(data d){

    //! Setup Kdl chain following the attached document : Kuka Kr 6 10 angilus.pdf
    KDLChain.addSegment(KDL::Segment("J0",KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(d.J0_x,d.J0_y,d.J0_z))));        //0.0 to J2
    KDLChain.addSegment(KDL::Segment("J1",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J1_x,d.J1_y,d.J1_z))));        //J2 to J3
    KDLChain.addSegment(KDL::Segment("J2",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J2_x,d.J2_y,d.J2_z))));        //J3 to J5
    KDLChain.addSegment(KDL::Segment("J3",KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(d.J3_x,d.J3_y,d.J3_z))));        //J4
    KDLChain.addSegment(KDL::Segment("J4",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J4_x,d.J4_y,d.J4_z))));        //J5 to end-effector (robot flange axis 6)
    KDLChain.addSegment(KDL::Segment("J5",KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(d.J5_x,d.J5_y,d.J5_z))));        //Tool cone lenght.

    // The floortrack is loaded outside the KDLChainVec size.

    KDLJointMin.resize(KDLChain.getNrOfSegments());
    KDLJointMax.resize(KDLChain.getNrOfSegments());
    KDLJointCur.resize(KDLChain.getNrOfSegments());
    KDLJointInit.resize(KDLChain.getNrOfSegments());

    KDLJointInit(0)= d.J0_init; //joint to radians
    KDLJointCur(0)= d.J0_cur;
    KDLJointMin(0)= d.J0_min;
    KDLJointMax(0)= d.J0_max;

    KDLJointInit(1)= d.J1_init; //joint to radians
    KDLJointCur(1)= d.J1_cur;
    KDLJointMin(1)= d.J1_min;
    KDLJointMax(1)= d.J1_max;

    KDLJointInit(2)= d.J2_init; //joint to radians
    KDLJointCur(2)= d.J2_cur;
    KDLJointMin(2)= d.J2_min;
    KDLJointMax(2)= d.J2_max;

    KDLJointInit(3)= d.J3_init; //joint to radians
    KDLJointCur(3)= d.J3_cur;
    KDLJointMin(3)= d.J3_min;
    KDLJointMax(3)= d.J3_max;

    KDLJointInit(4)= d.J4_init; //joint to radians
    KDLJointCur(4)= d.J4_cur;
    KDLJointMin(4)= d.J4_min;
    KDLJointMax(4)= d.J4_max;

    KDLJointInit(5)= d.J5_init; //joint to radians
    KDLJointCur(5)= d.J5_cur;
    KDLJointMin(5)= d.J5_min;
    KDLJointMax(5)= d.J5_max;

    // Perform a forward kinematic calculation (fk).
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(KDLChain);
    int status = fksolver.JntToCart(KDLJointCur,cart,-1);

    if(status==0 || status>0){

        // Add the xyz values to the data bucket.
        d.Cartx=cart.p.x();
        d.Carty=cart.p.y();
        d.Cartz=cart.p.z();

        // Add the euler values to the data bucket.
        cart.M.GetEulerZYX(d.Eulerz,d.Eulery,d.Eulerx);

        // Add the joint values to the data bucket.
        d.J0=KDLJointCur(0);
        d.J1=KDLJointCur(1);
        d.J2=KDLJointCur(2);
        d.J3=KDLJointCur(3);
        d.J4=KDLJointCur(4);
        d.J5=KDLJointCur(5);

    } else {
        //std::cout<<fksolver.getError()<<std::endl;
 
    }

    return d;
}

data Next::fk(data d){

    //! Setup Kdl chain following the attached document : Kuka Kr 6 10 angilus.pdf
    KDLChain.addSegment(KDL::Segment("J0",KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(d.J0_x,d.J0_y,d.J0_z))));        //0.0 to J2
    KDLChain.addSegment(KDL::Segment("J1",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J1_x,d.J1_y,d.J1_z))));        //J2 to J3
    KDLChain.addSegment(KDL::Segment("J2",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J2_x,d.J2_y,d.J2_z))));        //J3 to J5
    KDLChain.addSegment(KDL::Segment("J3",KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(d.J3_x,d.J3_y,d.J3_z))));        //J4
    KDLChain.addSegment(KDL::Segment("J4",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J4_x,d.J4_y,d.J4_z))));        //J5 to end-effector (robot flange axis 6)
    KDLChain.addSegment(KDL::Segment("J5",KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(d.J5_x,d.J5_y,d.J5_z))));        //Tool cone lenght.

    // The floortrack is loaded outside the KDLChainVec size.

    KDLJointMin.resize(KDLChain.getNrOfSegments());
    KDLJointMax.resize(KDLChain.getNrOfSegments());
    KDLJointCur.resize(KDLChain.getNrOfSegments());
    KDLJointInit.resize(KDLChain.getNrOfSegments());

    KDLJointInit(0)= d.J0_init; //joint to radians
    KDLJointCur(0)= d.J0;
    KDLJointMin(0)= d.J0_min;
    KDLJointMax(0)= d.J0_max;

    KDLJointInit(1)= d.J1_init; //joint to radians
    KDLJointCur(1)= d.J1;
    KDLJointMin(1)= d.J1_min;
    KDLJointMax(1)= d.J1_max;

    KDLJointInit(2)= d.J2_init; //joint to radians
    KDLJointCur(2)= d.J2;
    KDLJointMin(2)= d.J2_min;
    KDLJointMax(2)= d.J2_max;

    KDLJointInit(3)= d.J3_init; //joint to radians
    KDLJointCur(3)= d.J3;
    KDLJointMin(3)= d.J3_min;
    KDLJointMax(3)= d.J3_max;

    KDLJointInit(4)= d.J4_init; //joint to radians
    KDLJointCur(4)= d.J4;
    KDLJointMin(4)= d.J4_min;
    KDLJointMax(4)= d.J4_max;

    KDLJointInit(5)= d.J5_init; //joint to radians
    KDLJointCur(5)= d.J5;
    KDLJointMin(5)= d.J5_min;
    KDLJointMax(5)= d.J5_max;

    // Perform a forward kinematic calculation (fk).
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(KDLChain);
    int status = fksolver.JntToCart(KDLJointCur,cart,-1);

    if(status==0 || status>0){
     
        //std::cout<<"cartx:"<<cart.p.x()<<" y:"<<cart.p.y()<<" z:"<<cart.p.z()<<std::endl;

        // Add the xyz values to the data bucket.
        d.Cartx=cart.p.x();
        d.Carty=cart.p.y();
        d.Cartz=cart.p.z();

        // Add the euler values to the data bucket.
        cart.M.GetEulerZYX(d.Eulerz,d.Eulery,d.Eulerx);

    } else {
        //std::cout<<fksolver.getError()<<std::endl;
        
    }

    return d;
}

data Next::ik(data d){

    //! Setup Kdl chain following the attached document : Kuka Kr 6 10 angilus.pdf
    KDLChain.addSegment(KDL::Segment("J0",KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(d.J0_x,d.J0_y,d.J0_z))));        //0.0 to J2
    KDLChain.addSegment(KDL::Segment("J1",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J1_x,d.J1_y,d.J1_z))));        //J2 to J3
    KDLChain.addSegment(KDL::Segment("J2",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J2_x,d.J2_y,d.J2_z))));        //J3 to J5
    KDLChain.addSegment(KDL::Segment("J3",KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(d.J3_x,d.J3_y,d.J3_z))));        //J4
    KDLChain.addSegment(KDL::Segment("J4",KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(d.J4_x,d.J4_y,d.J4_z))));        //J5 to end-effector (robot flange axis 6)
    KDLChain.addSegment(KDL::Segment("J5",KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(d.J5_x,d.J5_y,d.J5_z))));        //Tool cone lenght.

    // The floortrack is loaded outside the KDLChainVec size.

    KDLJointMin.resize(KDLChain.getNrOfSegments());
    KDLJointMax.resize(KDLChain.getNrOfSegments());
    KDLJointCur.resize(KDLChain.getNrOfSegments());
    KDLJointInit.resize(KDLChain.getNrOfSegments());

    KDLJointInit(0)= d.J0_init; //joint to radians
    KDLJointCur(0)= d.J0;
    KDLJointMin(0)= d.J0_min;
    KDLJointMax(0)= d.J0_max;

    KDLJointInit(1)= d.J1_init; //joint to radians
    KDLJointCur(1)= d.J1;
    KDLJointMin(1)= d.J1_min;
    KDLJointMax(1)= d.J1_max;

    KDLJointInit(2)= d.J2_init; //joint to radians
    KDLJointCur(2)= d.J2;
    KDLJointMin(2)= d.J2_min;
    KDLJointMax(2)= d.J2_max;

    KDLJointInit(3)= d.J3_init; //joint to radians
    KDLJointCur(3)= d.J3;
    KDLJointMin(3)= d.J3_min;
    KDLJointMax(3)= d.J3_max;

    KDLJointInit(4)= d.J4_init; //joint to radians
    KDLJointCur(4)= d.J4;
    KDLJointMin(4)= d.J4_min;
    KDLJointMax(4)= d.J4_max;

    KDLJointInit(5)= d.J5_init; //joint to radians
    KDLJointCur(5)= d.J5;
    KDLJointMin(5)= d.J5_min;
    KDLJointMax(5)= d.J5_max;

    // Add the xyz values to the data bucket.
    cart.p.x(d.Cartx);
    cart.p.y(d.Carty);
    cart.p.z(d.Cartz);

    cart.M.DoRotX(d.Eulerx-cart.M.GetRot().x());
    cart.M.DoRotY(d.Eulery-cart.M.GetRot().y());
    cart.M.DoRotZ(d.Eulerz-cart.M.GetRot().z());

    // Perform a inverse kinematic calculation (fk).
    int status=0;
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(KDLChain);

    KDL::ChainIkSolverVel_pinv iksolverv(KDLChain); //Inverse velocity solver needed for IK
    KDL::ChainIkSolverPos_NR_JL iksolver(KDLChain, KDLJointMin, KDLJointMax, fksolver, iksolverv, 100, 1e-6); //max 100 iterations, stop at accuracy 1e-6
    // We can add a hal pin in the future to set max iteration values.

    KDL::JntArray JntResult(KDLChain.getNrOfJoints());

    //int status=0;
    bool mode_ikfrominit=1;
    if(mode_ikfrominit){ // Perform Ik from Init, we can add a hal input pin for this.
        status = iksolver.CartToJnt(KDLJointInit, cart, JntResult);
    } else {
        status = iksolver.CartToJnt(KDLJointCur, cart, JntResult);
    }

    if(status>=0){
        KDLJointCur=JntResult;

        // Write values to the bucket data.
        d.J0=KDLJointCur(0);
        d.J1=KDLJointCur(1);
        d.J2=KDLJointCur(2);
        d.J3=KDLJointCur(3);
        d.J4=KDLJointCur(4);
        d.J5=KDLJointCur(5);

    } else {
        //std::cout<<iksolver.getError()<<std::endl;
        
    }
    return d;
}

extern "C" data init_wrapper(data d){
    struct data r;
    r=Next().init(d);
    return r;
}

extern "C" data fk_wrapper(data d){
    struct data r;
    r=Next().fk(d);
    return r;
}

extern "C" data ik_wrapper(data d){
    struct data r;
    r=Next().ik(d);
    return r;
}
