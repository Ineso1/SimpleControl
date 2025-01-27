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

    GroupBox* reglages_groupbox=new GroupBox(position,name);
            kpatt=new Vector3DSpinBox(reglages_groupbox->NewRow(),"kpatt",0,100,1);
            kdatt=new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kdatt",0,100,1);
            satAtt=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"satAtt:",0,1,0.1);

            kppos=new Vector3DSpinBox(reglages_groupbox->NewRow(),"kppos",0,100,0.01);
            kdpos=new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kdpos",0,100,0.01);

			satPos=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"satPos:",0,1,0.1);
            satPosForce=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"satPosForce:",0,1,0.1);
            mg=new DoubleSpinBox(reglages_groupbox->NewRow(),"mg",0,100,0.0001);
}

Law::~Law(void) {
}

void Law::UseDefaultPlot(const gui::LayoutPosition* position) {
}

void Law::Reset(void) {
    p_d.x=0;
    p_d.y=0;
    p_d.z=0;
}

void Law::UpdateFrom(const io_data *data) {
    float delta_t,Fth;
    string currentstate;

    delta_t=(float)(data->DataTime()-previous_time)/1000000000.;

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

    float dt=0.005;
	float test;

    if(first_update==true) {
        delta_t=0;
        Fu.x=0;
        Fu.y=0;
        Fu.z=0.452;
        first_update=false;
    }

    Vector3Df pos_err=p-(p_d);
    Vector3Df vel_err=dp-(dp_d);
    Vector3Df ppos,dpos;
    
    std::cout<<"err p:\t"<<pos_err.x<<"\t"<<pos_err.y<<"\t"<<pos_err.z<<std::endl;
	if (pos_err.GetNorm()>satPos->Value()){
		Vector3Df sat_pos_err = pos_err*(satPos->Value()/pos_err.GetNorm());
        std::cout<<"Aqui stoy"<<std::endl;
    }

    ppos = -kppos->Value()*pos_err;
    dpos = -kdpos->Value()*vel_err;

    Fu = (ppos + dpos);

	Fu.Saturate(satPosForce->Value());
    Fu.z = Fu.z - 0.452;
    
	Fth = -Fu.GetNorm();

    if (Fth>0){Fth = 0;}

    Quaternion qt,qd;

    qz = Quaternion(1,0,0,0);

    if (Fu.GetNorm()!=0){
        qt.q0=DotProduct(Vector3Df(0,0,-1),Fu)+Fu.GetNorm();      
        Vector3Df tmp=CrossProduct(Vector3Df(0,0,-1),Fu);       
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
    Vector3Df Tau=Tau_u;
    Tau.Saturate(satAtt->Value());

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

    previous_time=data->DataTime();

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


} // end namespace filter
} // end namespace flair
