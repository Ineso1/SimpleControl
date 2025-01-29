#include "Law.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <GroupBox.h>
#include <Quaternion.h>
#include <Euler.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair
{
namespace filter
{
	
Law::Law(const LayoutPosition* position,string name) : ControlLaw(position->getLayout(),name,8) {

    input=new Matrix(this,23,1,floatType,name);
    desc=new MatrixDescriptor(23,1);
    desc->SetElementName(0,0,"q.q0");
    desc->SetElementName(1,0,"q.q1");
    desc->SetElementName(2,0,"q.q2");
    desc->SetElementName(3,0,"q.q3");
    desc->SetElementName(4,0,"qd.q0");
    desc->SetElementName(5,0,"qd.q1");
    desc->SetElementName(6,0,"qd.q2");
    desc->SetElementName(7,0,"qd.q3");
    desc->SetElementName(8,0,"w.x");
    desc->SetElementName(9,0,"w.y");
    desc->SetElementName(10,0,"w.z");
    desc->SetElementName(11,0,"p.x");
    desc->SetElementName(12,0,"p.y");
    desc->SetElementName(13,0,"p.z");
    desc->SetElementName(14,0,"p_d.x");
    desc->SetElementName(15,0,"p_d.y");
    desc->SetElementName(16,0,"p_d.z");
    desc->SetElementName(17,0,"dp.x");
    desc->SetElementName(18,0,"dp.y");
    desc->SetElementName(19,0,"dp.z");
    desc->SetElementName(20,0,"dp_d.x");
    desc->SetElementName(21,0,"dp_d.y");
    desc->SetElementName(22,0,"dp_d.z");
    stateM=new Matrix(this,desc,floatType,name);
    AddDataToLog(stateM);
    Reset();
    first_update=true;
    isDisturbanceActive = false;
    isDisturbanceRotActive = false;
    isKalmanActive = false;
    rejectionPercent = Vector3Df(0,0,0);
    rejectionRotPercent = Vector3Df(0,0,0);
    w_estimation_trans = Vector3Df(0,0,0);
    w_estimation_rot = Vector3Df(0,0,0);
    perturbation_trans = Vector3Df(0,0,0);
    u_thrust = Vector3Df(0,0,0);
    u_torque = Vector3Df(0,0,0);
    mass = 0.445;
    g = 9.81;
    observerMode = ObserverMode_t::UDE;

    previous_chrono_time = std::chrono::high_resolution_clock::now();

    p_d = Vector3Df(0,0,1);
    dp_d = Vector3Df(0,0,0);

    controlLayout = new GridLayout(position, "PD control");
    GroupBox* control_groupbox_att = new GroupBox(controlLayout->LastRowLastCol(),"Ude");
    kpatt=new Vector3DSpinBox(control_groupbox_att->NewRow(),"kp att",0,100,1);
    kdatt=new Vector3DSpinBox(control_groupbox_att->NewRow(),"kd att",0,100,1);
    satAtt=new DoubleSpinBox(control_groupbox_att->NewRow(),"sat att:",0,1,0.1);

    GroupBox* control_groupbox_trans = new GroupBox(controlLayout->LastRowLastCol(),"Translation");
    kppos=new Vector3DSpinBox(control_groupbox_trans->NewRow(),"kp pos",0,100,0.01);
    kdpos=new Vector3DSpinBox(control_groupbox_trans->NewRow(),"kd pos",0,100,0.01);
    satPos=new DoubleSpinBox(control_groupbox_trans->NewRow(),"satPos:",0,1,0.1);
    satPosForce=new DoubleSpinBox(control_groupbox_trans->NewRow(),"satPosForce:",0,1,0.1);
    
    paramsLayout = new GridLayout(controlLayout->NewRow(), "Params");
    GroupBox* params_groupbox = new GroupBox(paramsLayout->NewRow(),"Params");
    mg=new DoubleSpinBox(params_groupbox->NewRow(),"mg",0,100,0.0001);

    GridLayout* obsLayout = new GridLayout(controlLayout->NewRow(), "UDE");
    GroupBox* ude_groupbox = new GroupBox(obsLayout->LastRowLastCol(),"gains");
    omegaGainsTrans=new Vector3DSpinBox(ude_groupbox->NewRow(),"omega trans",0,100,0.1);
    omegaGainsRot=new Vector3DSpinBox(ude_groupbox->NewRow(),"omega rot",0,100,0.1);
    GroupBox* sm_groupbox = new GroupBox(obsLayout->LastRowLastCol(),"gains");    
    smUpperBoundTrans=new DoubleSpinBox(ude_groupbox->NewRow(),"upperBound trans",0,100,0.00001);
    smUpperBoundRot=new DoubleSpinBox(ude_groupbox->NewRow(),"upperBound rot",0,100,0.00001);
    smFilterTrans=new DoubleSpinBox(ude_groupbox->NewRow(),"filter trans",0,100,0.00001);
    smFilterRot=new DoubleSpinBox(ude_groupbox->NewRow(),"filter rot",0,100,0.00001);

    smCeTrans=new DoubleSpinBox(ude_groupbox->NewRow(),"ce trans",0,100,0.00001,10);
    smCeRot=new DoubleSpinBox(ude_groupbox->NewRow(),"ce rot",0,100,0.00001,7);
    smCdeTrans=new DoubleSpinBox(ude_groupbox->NewRow(),"cde trans",0,100,0.00001,10);
    smCdeRot=new DoubleSpinBox(ude_groupbox->NewRow(),"cde rot",0,100,0.00001,7);

}

Law::~Law(void) {
}

void Law::UseDefaultPlot(const gui::LayoutPosition* position) {
}

void Law::Reset(void) {
    p_d.x = 0;
    p_d.y = 0;
    p_d.z = 0;
    dp_d.x = 0;
    dp_d.y = 0;
    dp_d.z = 0;
}

void Law::UpdateFrom(const io_data *data) {
    float Fth, dt;
    string currentstate;

    input->GetMutex();
    Quaternion q(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0),input->ValueNoMutex(3,0));
    q.Normalize();
    Quaternion qz(input->ValueNoMutex(4,0),input->ValueNoMutex(5,0),input->ValueNoMutex(6,0),input->ValueNoMutex(7,0));
    Vector3Df w(input->ValueNoMutex(8,0),input->ValueNoMutex(9,0),input->ValueNoMutex(10,0));
    Vector3Df p(input->ValueNoMutex(11,0),input->ValueNoMutex(12,0),input->ValueNoMutex(13,0));
    Vector3Df p_d(input->ValueNoMutex(14,0),input->ValueNoMutex(15,0),input->ValueNoMutex(16,0));
    Vector3Df dp(input->ValueNoMutex(17,0),input->ValueNoMutex(18,0),input->ValueNoMutex(19,0));
    Vector3Df dp_d(input->ValueNoMutex(20,0),input->ValueNoMutex(21,0),input->ValueNoMutex(22,0));
    input->ReleaseMutex();

    Vector3Df pos_err=p-(p_d);
    Vector3Df vel_err=dp-(dp_d);
    Vector3Df ppos,dpos;
    
    if(first_update==true) {
        previous_chrono_time = std::chrono::high_resolution_clock::now();
        dt=0;
        Fu.x=0;
        Fu.y=0;
        Fu.z=0.452;
        first_update=false;
    }


    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> alt_dt = current_time - previous_chrono_time;
    dt = alt_dt.count();

    if(observerMode == Law::ObserverMode_t::UDE){
        ude_obs.Omega_UDE_trans = (Eigen::Matrix3f() << 
                                omegaGainsTrans->Value().x, 0.0f, 0.0f,
                                0.0f, omegaGainsTrans->Value().y, 0.0f,
                                0.0f, 0.0f, omegaGainsTrans->Value().z).finished();
        ude_obs.Omega_UDE_rot = (Eigen::Matrix3f() << 
                                omegaGainsRot->Value().x, 0.0f, 0.0f,
                                0.0f, omegaGainsRot->Value().y, 0.0f,
                                0.0f, 0.0f, omegaGainsRot->Value().z).finished();
        w_estimation_trans = ude_obs.EstimateDisturbance_trans(p, dp, u_thrust, dt);
        w_estimation_rot = ude_obs.EstimateDisturbance_rot(q, w, u_torque, dt);
    } else if(observerMode == Law::ObserverMode_t::Luenberger){
        // w_estimation_trans = luenberger.EstimateDisturbance_trans(p, dp, u_thrust, dt);
        // w_estimation_rot = luenberger.EstimateDisturbance_rot(q, w, u_torque, dt);
    } else if(observerMode == Law::ObserverMode_t::SuperTwist){
        // w_estimation_trans = superTwist.EstimateDisturbance_trans(p, dp, u_thrust, dt);
        // w_estimation_rot = superTwist.EstimateDisturbance_rot(q, w, u_torque, dt);
    }else if(observerMode == Law::ObserverMode_t::SlidingMode){
        slidingMode_obs.SetUpperBounds(smUpperBoundTrans->Value(), smUpperBoundRot->Value());
        slidingMode_obs.SetFilterGains(smFilterTrans->Value(), smFilterRot->Value());
        slidingMode_obs.SetSlidingGains(smCeTrans->Value(), smCdeTrans->Value(), smCeRot->Value(), smCdeRot->Value());
        w_estimation_trans = slidingMode_obs.EstimateDisturbance_trans(p, dp, u_thrust, dt);
        w_estimation_rot = slidingMode_obs.EstimateDisturbance_rot(q, w, u_torque, dt);
    }


    if(!isDisturbanceActive){
        w_estimation_trans = Vector3Df(0,0,0); 
    }

    if(!isDisturbanceRotActive){
        w_estimation_rot = Vector3Df(0,0,0);
    }

    w_estimation_trans = Vector3Df(w_estimation_trans.x * rejectionPercent.x, w_estimation_trans.y * rejectionPercent.y, w_estimation_trans.z * rejectionPercent.z);
    w_estimation_rot = Vector3Df(w_estimation_rot.x * rejectionRotPercent.x, w_estimation_rot.y * rejectionRotPercent.y, w_estimation_rot.z * rejectionRotPercent.z);

    std::cout<< "err\t" << pos_err.x << "\t" << pos_err.y << "\t" << pos_err.z << std::endl;
    std::cout<< "w_t\t" << w_estimation_trans.x << "\t" << w_estimation_trans.y << "\t" << w_estimation_trans.z << std::endl;
    std::cout<< "w_r\t" << w_estimation_rot.x << "\t" << w_estimation_rot.y << "\t" << w_estimation_rot.z << std::endl;
    

	if (pos_err.GetNorm()>satPos->Value()){
		Vector3Df sat_pos_err = pos_err*(satPos->Value()/pos_err.GetNorm());
    }

    ppos = -kppos->Value()*pos_err;
    dpos = -kdpos->Value()*vel_err;

    Fu = (ppos + dpos);
    Fu = Fu - w_estimation_trans;

	Fu.Saturate(satPosForce->Value());
    Fu.z = Fu.z - (mass * g /10);

    Vector3Df Fu_perturbed = Fu - perturbation_trans;
    
	Fth = -Fu_perturbed.GetNorm();

    if (Fth>0){Fth = 0;}

    Quaternion qt,qd;

    qz = Quaternion(1,0,0,0);

    if (Fu_perturbed.GetNorm()!=0){
        qt.q0=DotProduct(Vector3Df(0,0,-1),Fu_perturbed)+Fu_perturbed.GetNorm();      
        Vector3Df tmp=CrossProduct(Vector3Df(0,0,-1),Fu_perturbed);       
        qt.q1=tmp.x;
        qt.q2=tmp.y;
        qt.q3=tmp.z;
        qt.Normalize();
        qd = qt*qz;
        qd.Normalize();
    }else{
        qt = q;
        qd = qt;
        qd.Normalize();
    }
	Quaternion qe = qd.GetConjugate()*q;
	Vector3Df thetae = 2*qe.GetLogarithm();

    qe = qd.GetConjugate()*q;
    thetae=2*qe.GetLogarithm();

    Vector3Df patt,datt;
    patt = kpatt->Value()*thetae;
    datt = kdatt->Value()*w;

    Vector3Df Tau_u = patt + datt;
    Vector3Df Tau=Tau_u - w_estimation_rot;
    Tau.Saturate(satAtt->Value());

    u_thrust = Fu;
    u_torque = Tau;

    output->SetValue(0,0,Tau.x);
    output->SetValue(1,0,Tau.y);
    output->SetValue(2,0,Tau.z);
    output->SetValue(3,0,Fth);
    output->SetDataTime(data->DataTime());
	
	output->SetValue(4,0,qd.q0);
    output->SetValue(5,0,qd.q1);
    output->SetValue(6,0,qd.q2);
	output->SetValue(7,0,qd.q3);

    stateM->GetMutex();
    stateM->SetValueNoMutex(0,0,q.q0);
    stateM->SetValueNoMutex(1,0,q.q1);
    stateM->SetValueNoMutex(2,0,q.q2);
    stateM->SetValueNoMutex(3,0,q.q3);
    stateM->SetValueNoMutex(4,0,qd.q0);
    stateM->SetValueNoMutex(5,0,qd.q1);
    stateM->SetValueNoMutex(6,0,qd.q2);
    stateM->SetValueNoMutex(7,0,qd.q3);
    stateM->SetValueNoMutex(8,0,w.x);
    stateM->SetValueNoMutex(9,0,w.y);
    stateM->SetValueNoMutex(10,0,w.z);
    stateM->SetValueNoMutex(11,0,p.x);
    stateM->SetValueNoMutex(12,0,p.y);
    stateM->SetValueNoMutex(13,0,p.z);
    stateM->SetValueNoMutex(14,0,p_d.x);
    stateM->SetValueNoMutex(15,0,p_d.y);
    stateM->SetValueNoMutex(16,0,p_d.z);
    stateM->SetValueNoMutex(17,0,dp.x);
    stateM->SetValueNoMutex(18,0,dp.y);
    stateM->SetValueNoMutex(19,0,dp.z);
    stateM->SetValueNoMutex(20,0,dp_d.x);
    stateM->SetValueNoMutex(21,0,dp_d.y);
    stateM->SetValueNoMutex(22,0,dp_d.z);
    stateM->ReleaseMutex();

    previous_chrono_time = current_time;

    ProcessUpdate(output);
}

void Law::SetValues(Quaternion q,Quaternion qd,Vector3Df w,core::Vector3Df p,core::Vector3Df p_d,core::Vector3Df dp,core::Vector3Df dp_d) {
    input->SetValue(0,0,q.q0);
    input->SetValue(1,0,q.q1);
    input->SetValue(2,0,q.q2);
    input->SetValue(3,0,q.q3);
    input->SetValue(4,0,qd.q0);
    input->SetValue(5,0,qd.q1);
    input->SetValue(6,0,qd.q2);
    input->SetValue(7,0,qd.q3);
    input->SetValue(8,0,w.x);
    input->SetValue(9,0,w.y);
    input->SetValue(10,0,w.z);
    input->SetValue(11,0,p.x);
    input->SetValue(12,0,p.y);
    input->SetValue(13,0,p.z);
    input->SetValue(14,0,p_d.x);
    input->SetValue(15,0,p_d.y);
    input->SetValue(16,0,p_d.z);
    input->SetValue(17,0,dp.x);
    input->SetValue(18,0,dp.y);
    input->SetValue(19,0,dp.z);
    input->SetValue(20,0,dp_d.x);
    input->SetValue(21,0,dp_d.y);
    input->SetValue(22,0,dp_d.z);
}


void Law::SetTarget(Vector3Df target_pos, Vector3Df target_vel, Quaternion target_yaw){
    p_d = target_pos;
}

void Law::SetRejectionPercent(Vector3Df rejection, Vector3Df rejectionRot){
    rejectionPercent = rejection;
    rejectionRotPercent = rejectionRot;
}

void Law::SetPerturbation(Vector3Df p_trans, Vector3Df p_rot){
    perturbation_trans = p_trans;
}

} // end namespace filter
} // end namespace flair
