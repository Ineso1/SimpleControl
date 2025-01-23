// Drone.cpp
#include "Drone.h"

Drone::Drone(TargetController *controller) : DroneBase(controller) {
    
    //Behave
    algorithmBehaviourMode = AlgorithmBehaviourMode_t::PositionPoint;
    sequenceFirstTime = true;

    // Toggle variables
    kalman = false;
    perturbation = false;

    // Law instance
    myLaw = new MyLaw(execLayout->At(1, 0), "MyLaw");

    // Set Zero Orientation 
    initQuaternion =  GetCurrentQuaternion();
    customReferenceOrientation= new AhrsData(this,"reference");

    // Set current position target
    flair::core::Vector3Df uav_p;
    uavVrpn->GetPosition(uav_p);
    currentTarget = Vector3Df(uav_p.x,uav_p.y,1);
}

Drone::~Drone() {
    delete myLaw;
}

/***************************************** 
 * Buttons Handlers
*****************************************/

void Drone::PositionChange(void) {
    if(algorithmBehaviourMode == AlgorithmBehaviourMode_t::PositionPoint){
        currentTarget = flair::core::Vector3Df(targetPosition_layout->Value().x, targetPosition_layout->Value().y, targetPosition_layout->Value().z);
    }
}

void Drone::HandleDisturbanceToggle() {
    perturbation = !perturbation;
    if (perturbation) {
        Vector3Df perturbationVec = flair::core::Vector3Df(perturbation_layout->Value().x, perturbation_layout->Value().y, perturbation_layout->Value().z);
        myLaw->SetPerturbation(perturbationVec, Vector3Df(0, 0, 0));
        disturbanceModeState->SetText("state: on +++++");
    } else {
        myLaw->SetPerturbation(Vector3Df(0, 0, 0), Vector3Df(0, 0, 0));
        disturbanceModeState->SetText("state: ----- off");
    }
}

void Drone::RejectDisturbance() {
    myLaw->isDisturbanceActive = !myLaw->isDisturbanceActive;
    if(myLaw->isDisturbanceActive){
        rejectionModeState->SetText("state: on +++++");
    }
    else{
        rejectionModeState->SetText("state: ----- off");
    }
}

void Drone::RejectRotDisturbance() {
    myLaw->isDisturbanceRotActive = !myLaw->isDisturbanceRotActive;
    if(myLaw->isDisturbanceRotActive){
        rejectionRotModeState->SetText("state: on +++++");
    }
    else{
        rejectionRotModeState->SetText("state: ----- off");
    }
}

void Drone::ApplyKalman() {
    kalman = !kalman;
    myLaw->isKalmanActive = kalman;
    if(kalman){
        kalmanActivationState->SetText("state: on +++++");
    }
    else{
        kalmanActivationState->SetText("state: ----- off");
    }
}

/***************************************** 
 * Control behave
*****************************************/

void Drone::ApplyControl() {
    switch(observerMode_layout->CurrentIndex()){
        case 0:
            myLaw->observerMode = MyLaw::ObserverMode_t::UDE;
            break;
        case 1:
            myLaw->observerMode = MyLaw::ObserverMode_t::Luenberger;
            break;
        case 2:
            myLaw->observerMode = MyLaw::ObserverMode_t::SuperTwist;
            break;
        case 3:
            myLaw->observerMode = MyLaw::ObserverMode_t::SlidingMode;
            break;
    }

    switch(beahviourMode_layout->CurrentIndex()){
        case 0:
            PositionControl();
            break;
        case 1:
            TargetFollowControl();
            break;
        case 2:
            TestObserverSequence();
            break;
    }
}

/***************************************** 
 * Correction Functions
*****************************************/

void Drone::MixOrientation() {
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

/***************************************** 
 * Control inputs
*****************************************/

float Drone::ComputeCustomThrust() {
    return 0;
}


const AhrsData *Drone::GetOrientation(void) const {
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


AhrsData *Drone::GetReferenceOrientation(void) {
    float yaw_ref;
    Euler refAngles;

    Vector3Df uav_p, uav_dp; 
    Vector3Df aim_p, aim_dp;
    Quaternion uav_q;
    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);
    uav_q = GetCurrentQuaternion();

    yaw_ref = 0;
    refAngles.yaw=yaw_ref;

    myLaw->UpdateTranslationControl(uav_p, uav_dp, uav_q);
    refAngles.pitch=myLaw->uX_custom->Output();
    refAngles.roll=-myLaw->uY_custom->Output();

    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));

    return customReferenceOrientation;
}



/***************************************** 
 * Control behave algorithm functions
*****************************************/

void Drone::PositionControl(){
    Vector3Df uav_p, uav_dp; 
    Vector3Df aim_p, aim_dp;
    Vector3Df rejectionPercent = flair::core::Vector3Df(rejectionPercent_layout->Value().x, rejectionPercent_layout->Value().y, rejectionPercent_layout->Value().z);
    Quaternion q = GetCurrentQuaternion();
    Vector3Df w;
    Quaternion aim_yaw(1,0,0,0);

    aim_p = currentTarget;
    aim_dp = Vector3Df(0, 0, 0);

    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);

    MixOrientation();
    q = mixQuaternion;
    w = mixAngSpeed;

    myLaw->SetRejectionPercent(rejectionPercent);
    myLaw->SetTarget(aim_p, aim_dp, aim_yaw);
    // myLaw->UpdateDynamics(uav_p, uav_dp, q, w);
    // myLaw->Update(GetTime());
}




void Drone::TargetFollowControl(){
    
}

void Drone::TestObserverSequence(void){
   

}