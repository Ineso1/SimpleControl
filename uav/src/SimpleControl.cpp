#include "SimpleControl.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

SimpleControl::SimpleControl(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        targetVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    }
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
																 
    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"start_circle");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_circle");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");

    circle=new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),"circle");
    uavVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    uavVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    uavVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    uavVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    uavVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");

    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());
    uZ_aux = new PidThrust(setupLawTab->At(1,3), "u_z_aux");
    uZ_aux->UseDefaultPlot(graphLawTab->LastRowLastCol());


    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);
    AddDeviceToControlLawLog(uZ_aux);

    customOrientation=new AhrsData(this,"orientation");

    #ifdef CSV_STATE
        InitStateCSV();
    #endif

    refAltitude = 1;
    refVerticalVelocity = 0;

    
}

SimpleControl::~SimpleControl() {
}

const AhrsData *SimpleControl::GetOrientation(void) const {
    //get yaw from vrpn
    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);

    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    return customOrientation;
}

void SimpleControl::AltitudeValues(float &z,float &dz) const{
    Vector3Df p,dp;

    uavVrpn->GetPosition(p);
    uavVrpn->GetSpeed(dp);
    //z and dz must be in uav's frame
    z=-p.z;
    dz=-dp.z;
}

AhrsData *SimpleControl::GetReferenceOrientation(void) {
    Vector2Df pos_err, vel_err; // in Uav coordinate system
    float yaw_ref;
    Euler refAngles;
    Vector3Df p,dp; // in VRPN coordinate system

    /*******************
     *
     * Feedback vars
     *  
     *******************/


    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    Quaternion q;
    Vector3Df omega;

    uavVrpn->GetPosition(p);
    uavVrpn->GetSpeed(dp);
    MixOrientation();
    q = mixQuaternion;
    omega = mixAngSpeed;

    SaveStateCSV(p, dp, q, omega);

    PidThrust *uZ_pub = GetUZ();
    Pid* uYaw_pub = GetUYaw();
    NestedSat* uRoll_pub = GetURoll();
    NestedSat* uPitch_pub = GetUPitch();

    float uZ_val = uZ_pub->Output();
    float uYaw_val = uYaw_pub->Output();
    float uPitch_val = uPitch_pub->Output();
    float uRoll_val = uRoll_pub->Output();

    /*******************/

    PositionValues(pos_err, vel_err, yaw_ref);

    refAngles.yaw=yaw_ref;

    uX->SetValues(pos_err.x, vel_err.x);
    uX->Update(GetTime());
    refAngles.pitch=uX->Output();

    uY->SetValues(pos_err.y, vel_err.y);
    uY->Update(GetTime());
    refAngles.roll=-uY->Output();

    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));

    return customReferenceOrientation;
}

void SimpleControl::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref) {
    Vector3Df p,dp; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(p);
    uavVrpn->GetSpeed(dp);

    p.To2Dxy(uav_2Dpos);
    dp.To2Dxy(uav_2Dvel);

    if (behaviourMode==BehaviourMode_t::PositionHold) {
        pos_error=uav_2Dpos-posHold;
        vel_error=uav_2Dvel;
        yaw_ref=yawHold;
    } else { //Circle
        Vector3Df target_pos;
        Vector2Df circle_pos,circle_vel;
        Vector2Df target_2Dpos;

        targetVrpn->GetPosition(target_pos);
        target_pos.To2Dxy(target_2Dpos);
        circle->SetCenter(target_2Dpos);

        //circle reference
        circle->Update(GetTime());
        circle->GetPosition(circle_pos);
        circle->GetSpeed(circle_vel);

        //error in optitrack frame
        pos_error=uav_2Dpos-circle_pos;
        vel_error=uav_2Dvel-circle_vel;
        yaw_ref=atan2(target_pos.y-p.y,target_pos.x-p.x);
    }

    //error in uav frame
    Quaternion currentQuaternion=GetCurrentQuaternion();
    Euler currentAngles;//in vrpn frame
    currentQuaternion.ToEuler(currentAngles);
    pos_error.Rotate(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);

}

void SimpleControl::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode==BehaviourMode_t::Circle) && (!circle->IsRunning())) {
            VrpnPositionHold();
        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void SimpleControl::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void SimpleControl::ExtraCheckPushButton(void) {
    if(startCircle->Clicked()) {
        StartCircle();
    }
    if(stopCircle->Clicked()) {
        StopCircle();
    }
    if(positionHold->Clicked()) {
        VrpnPositionHold();
    }
}

void SimpleControl::ExtraCheckJoystick(void) {
    //R1 and Circle
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        StartCircle();
    }

    //R1 and Cross
    if(GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
        StopCircle();
    }
    
    //R1 and Square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        VrpnPositionHold();
    }
}

void SimpleControl::StartCircle(void) {
    if( behaviourMode==BehaviourMode_t::Circle) {
        Thread::Warn("SimpleControl: already in circle mode\n");
        return;
    }
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("SimpleControl: start circle\n");
    } else {
        Thread::Warn("SimpleControl: could not start circle\n");
        return;
    }
    if(SetThrustMode(ThrustMode_t::Custom)){
        std::cout<<"Thrust Custom"<<std::endl;
    } else {
        std::cout<<"Thrust no se puede custom"<<std::endl;
    }

    Vector3Df p,target_pos;
    Vector2Df uav_2Dpos,target_2Dpos;

    targetVrpn->GetPosition(target_pos);
    target_pos.To2Dxy(target_2Dpos);
    circle->SetCenter(target_2Dpos);

    uavVrpn->GetPosition(p);
    p.To2Dxy(uav_2Dpos);
    circle->StartTraj(uav_2Dpos);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::Circle;
}

void SimpleControl::StopCircle(void) {
    if( behaviourMode!=BehaviourMode_t::Circle) {
        Thread::Warn("SimpleControl: not in circle mode\n");
        return;
    }
    circle->FinishTraj();
    //GetJoystick()->Rumble(0x70);
    Thread::Info("SimpleControl: finishing circle\n");
}

void SimpleControl::VrpnPositionHold(void) {
    if( behaviourMode==BehaviourMode_t::PositionHold) {
        Thread::Warn("SimpleControl: already in vrpn position hold mode\n");
        return;
    }
		Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
		yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("SimpleControl: holding position\n");
}


void SimpleControl::SaveStateCSV(flair::core::Vector3Df &p, flair::core::Vector3Df &dp, flair::core::Quaternion &q, flair::core::Vector3Df &omega){
    std::ofstream fileStateCSV("/home/nessy/flair/flair-src/myDemos/SimpleControl/DataCSV/State.csv", std::ios::app);
    if (fileStateCSV.is_open()) {
        fileStateCSV    << p.x << "," << p.y << "," << p.z << "," 
                        << dp.x << "," << dp.y << "," << dp.z << "," 
                        << q.q0 << "," << q.q1 << "," << q.q2 << "," << q.q3 << ","
                        << omega.x << "," << omega.y << "," << omega.z
                        << std::endl;
    }
    else{
        std::cout<<"..."<<std::endl;
    }
}

void SimpleControl::InitStateCSV(){
    std::ofstream fileStateCSV("/home/nessy/flair/flair-src/myDemos/SimpleControl/DataCSV/State.csv", std::ios::app);
    if (fileStateCSV.is_open()) {
        fileStateCSV << "p_x" << "," << "p_y" << "," << "p_z" << "," << "dp_x" << "," << "dp_y" << "," << "dp_z" << "," << "q_w" << ","  << "q_x" << "," << "q_y" << "," << "q_z" << ","  << "omega_x" << "," << "omega_y" << "," << "omega_z" << std::endl;
    }
    else{
        std::cout<<"No se puede "<<std::endl;
    }
}

void SimpleControl::MixOrientation() {
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    if (first_up) {
        Quaternion vrpnQuaternion;
        uavVrpn->GetQuaternion(vrpnQuaternion);
        Euler vrpnEuler;
        vrpnQuaternion.ToEuler(vrpnEuler);
        vrpnQuaternion.q0 = std::cos(vrpnEuler.yaw / 2);
        vrpnQuaternion.q1 = 0;
        vrpnQuaternion.q2 = 0;
        vrpnQuaternion.q3 = std::sin(vrpnEuler.yaw / 2);
        vrpnQuaternion.Normalize();
        qI = vrpnQuaternion * ahrsQuaternion.GetConjugate();
        first_up = false;
    }

    mixQuaternion = qI * ahrsQuaternion;
    mixQuaternion.Normalize();
    mixAngSpeed = ahrsAngularSpeed;
}


float SimpleControl::ComputeCustomThrust(){
    // compute desired altitude
    float currentAltitude, currentVerticalSpeed;
    AltitudeValues(currentAltitude, currentVerticalSpeed);
    float p_error = currentAltitude - refAltitude;
    float d_error = currentVerticalSpeed - refVerticalVelocity;
    uZ_aux->SetValues(p_error, d_error);
    uZ_aux->Update(GetTime());
    float savedDefaultThrust = uZ_aux->Output();
    return savedDefaultThrust;
}
