//  created:    2011/05/01
//  filename:   Law.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/

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
     //init matrix
    input=new Matrix(this,23,1,floatType,name);
    
    MatrixDescriptor* desc=new MatrixDescriptor(23,1);
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
    desc->SetElementName(11,0,"pos.x");
    desc->SetElementName(12,0,"pos.y");
    desc->SetElementName(13,0,"pos.z");
    desc->SetElementName(14,0,"posd.x");
    desc->SetElementName(15,0,"posd.y");
    desc->SetElementName(16,0,"posd.z");
    desc->SetElementName(17,0,"vel.x");
    desc->SetElementName(18,0,"vel.y");
    desc->SetElementName(19,0,"vel.z");
    desc->SetElementName(20,0,"veld.x");
    desc->SetElementName(21,0,"veld.y");
    desc->SetElementName(22,0,"veld.z");
    stateM=new Matrix(this,desc,floatType,name);

    

    AddDataToLog(stateM);


    Reset();
    //first_update=true;

    GroupBox* reglages_groupbox=new GroupBox(position,name);
            T=new DoubleSpinBox(reglages_groupbox->NewRow(),"period, 0 for auto"," s",0,1,0.01);
            kpatt=new Vector3DSpinBox(reglages_groupbox->NewRow(),"kpatt",0,100,1);
            kdatt=new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kdatt",0,100,1);
            satAtt=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"satAtt:",0,1,0.1);
            PosRefRate=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"PosRefRate:",0,1,0.1);

            kppos=new Vector3DSpinBox(reglages_groupbox->NewRow(),"kppos",0,100,0.01);
            kdpos=new Vector3DSpinBox(reglages_groupbox->LastRowLastCol(),"kdpos",0,100,0.01);
            omg=new Vector3DSpinBox(reglages_groupbox->NewRow(),"omega",0,100,0.01);
			satPos=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"satPos:",0,1,0.1);
            satPosForce=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"satPosForce:",0,1,0.1);
            mg=new DoubleSpinBox(reglages_groupbox->NewRow(),"mg",0,100,0.0001);
            CustomDeltaT=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"CustomDeltaT:",0,1,0.1);
}

Law::~Law(void) {
}

void Law::UseDefaultPlot(const gui::LayoutPosition* position) {
}

void Law::Reset(void) {
    posd.x=0;
    posd.y=0;
    posd.z=0;
}

void Law::UpdateFrom(const io_data *data) {
    float delta_t,Fth;
    string currentstate;

    if(T->Value()==0) {
        delta_t=(float)(data->DataTime()-previous_time)/1000000000.;
    } else {
        delta_t=T->Value();
    }

    input->GetMutex();
    Quaternion q(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0),input->ValueNoMutex(3,0));
    q.Normalize();
    Quaternion qz(input->ValueNoMutex(4,0),input->ValueNoMutex(5,0),input->ValueNoMutex(6,0),input->ValueNoMutex(7,0));
    Vector3Df w(input->ValueNoMutex(8,0),input->ValueNoMutex(9,0),input->ValueNoMutex(10,0));
    Vector3Df pos(input->ValueNoMutex(11,0),input->ValueNoMutex(12,0),input->ValueNoMutex(13,0));
    Vector3Df posd(input->ValueNoMutex(14,0),input->ValueNoMutex(15,0),input->ValueNoMutex(16,0));
    Vector3Df vel(input->ValueNoMutex(17,0),input->ValueNoMutex(18,0),input->ValueNoMutex(19,0));
    Vector3Df veld(input->ValueNoMutex(20,0),input->ValueNoMutex(21,0),input->ValueNoMutex(22,0));
    input->ReleaseMutex();
    Vector3Df omega=omg->Value();

    float dt=0.005;
	float test;

    if(first_update==true) {
        delta_t=0;
		timeT = 0;
        // Eps=(omega*vel)/(0.397918);
        Eps=0;
        Fu.x=0;
        Fu.y=0;
        Fu.z=0.452;
        first_update=false;
    }

    // Loi de commande pour la position
    Vector3Df pos_err=pos-(posd);
    Vector3Df vel_err=vel-(veld);
    Vector3Df ppos,dpos;
    
    std::cout<<"err p:\t"<<pos_err.x<<"\t"<<pos_err.y<<"\t"<<pos_err.z<<std::endl;
	if (pos_err.GetNorm()>satPos->Value()){
		Vector3Df sat_pos_err = pos_err*(satPos->Value()/pos_err.GetNorm());
        std::cout<<"Aqui stoy"<<std::endl;
    }

    // // sim
    // Epsd.x=-omega.x*Eps.x-omega.x*omega.x*(vel_err.x*(1.2))-omega.x*(Fu.x);
    // Epsd.y=-omega.y*Eps.y-omega.y*omega.y*(vel_err.y*(1.2))-omega.y*(Fu.y);
    // Epsd.z=-omega.z*Eps.z-omega.z*omega.z*(vel_err.z*(1.2))-omega.z*(Fu.z+0.397918);
    // Eps=Eps+Epsd*dt;
    // west.x=(Eps.x+omega.x*(vel_err.x*(1.2)));
    // west.y=(Eps.y+omega.y*(vel_err.y*(1.2)));
    // west.z=(Eps.z+omega.z*(vel_err.z*(1.2)));

    //real world
    Epsd.x=-omega.x*Eps.x-omega.x*omega.x*(vel_err.x*(0.485))-omega.x*(Fu.x);
    Epsd.y=-omega.y*Eps.y-omega.y*omega.y*(vel_err.y*(0.485))-omega.y*(Fu.y);
    Epsd.z=-omega.z*Eps.z-omega.z*omega.z*(vel_err.z*(0.485))-omega.z*(Fu.z+0.452);
    Eps=Eps+Epsd*dt;
    west.x=(Eps.x+omega.x*(vel_err.x*(0.485)));
    west.y=(Eps.y+omega.y*(vel_err.y*(0.485)));
    west.z=(Eps.z+omega.z*(vel_err.z*(0.485)));


    // cout << "Fu z" << Fu.z << endl;
    // cout << "test" << test<< endl;

    // cout << "est" << west.z << endl;
    ppos = -kppos->Value()*pos_err;
    dpos = -kdpos->Value()*vel_err;

    Fu = (ppos + dpos) - west*0;

	Fu.Saturate(satPosForce->Value());
    // // sim
    // Fu.z = Fu.z - 0.397918; 
    // rear world
    Fu.z = Fu.z - 0.452;
 
    
	Fth = -Fu.GetNorm();

    
    
    // cout << "Fu z" << Fu.z << endl;



    if (Fth>0){Fth = 0;}

    Quaternion qt,qd;

    qz = Quaternion(1,0,0,0);


    if (Fu.GetNorm()!=0){
		
        qt.q0=DotProduct(Vector3Df(0,0,-1),Fu)+Fu.GetNorm();      //scalar part
        Vector3Df tmp=CrossProduct(Vector3Df(0,0,-1),Fu);        //vector part
        qt.q1=tmp.x;
        qt.q2=tmp.y;
        qt.q3=tmp.z;
        qt.Normalize();
        qd = qt*qz;
        qd.Normalize();
    }else{
        // printf("Thrust equals ZERO!!");
        qt = q;
        qd = qt;
        qd.Normalize();
    }
	Quaternion qe = qd.GetConjugate()*q;
	Vector3Df thetae = 2*qe.GetLogarithm();
	// float quatsign = 1;
	// if (thetae.GetNorm()>=3.1416){
	// 	printf("Attitude trajectory is too far!! %f \n",thetae.GetNorm());
	// 	quatsign = -1;
	// }
	// qd = quatsign*qd;
    


    // Loi de commande pour l'orientation
    qe = qd.GetConjugate()*q;
    thetae=2*qe.GetLogarithm();

    Vector3Df patt,datt;
    patt = kpatt->Value()*thetae;
    datt = kdatt->Value()*w;

    Vector3Df Tau_u = patt + datt;
    Vector3Df Tau=Tau_u;
    Tau.Saturate(satAtt->Value());

    // cout << "Tau x" << Tau.x << endl;
    // cout << "Tau y" << Tau.y << endl;
    // cout << "Tau z" << Tau.z << endl;

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
    stateM->SetValueNoMutex(11,0,pos.x);
    stateM->SetValueNoMutex(12,0,pos.y);
    stateM->SetValueNoMutex(13,0,pos.z);
    stateM->SetValueNoMutex(14,0,posd.x);
    stateM->SetValueNoMutex(15,0,posd.y);
    stateM->SetValueNoMutex(16,0,posd.z);
    stateM->SetValueNoMutex(17,0,vel.x);
    stateM->SetValueNoMutex(18,0,vel.y);
    stateM->SetValueNoMutex(19,0,vel.z);
    stateM->SetValueNoMutex(20,0,veld.x);
    stateM->SetValueNoMutex(21,0,veld.y);
    stateM->SetValueNoMutex(22,0,veld.z);
    stateM->ReleaseMutex();

    previous_time=data->DataTime();

    ProcessUpdate(output);
}

void Law::SetValues(Quaternion q,Quaternion qd,Vector3Df w,core::Vector3Df pos,core::Vector3Df posd,core::Vector3Df vel,core::Vector3Df veld) {
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
    input->SetValue(11,0,pos.x);
    input->SetValue(12,0,pos.y);
    input->SetValue(13,0,pos.z);
    input->SetValue(14,0,posd.x);
    input->SetValue(15,0,posd.y);
    input->SetValue(16,0,posd.z);
    input->SetValue(17,0,vel.x);
    input->SetValue(18,0,vel.y);
    input->SetValue(19,0,vel.z);
    input->SetValue(20,0,veld.x);
    input->SetValue(21,0,veld.y);
    input->SetValue(22,0,veld.z);
}


} // end namespace filter
} // end namespace flair
