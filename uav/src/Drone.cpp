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


/***************************************** 
 * Control behave algorithm functions
*****************************************/

void Drone::PositionControl(){
    Vector3Df ref_p(0, 0, 0);
    Vector3Df uav_p, uav_dp; 
    Vector3Df aim_p, aim_dp;
    Vector3Df rejectionPercent = flair::core::Vector3Df(rejectionPercent_layout->Value().x, rejectionPercent_layout->Value().y, rejectionPercent_layout->Value().z);
    Quaternion q = GetCurrentQuaternion();
    Vector3Df w;
    yawAngle = yawAngle_layout->Value();
    float yawAngleInRadians = yawAngle * M_PI / 180.0;
    Quaternion aim_yaw(std::cos(yawAngleInRadians / 2), 0, 0, std::sin(yawAngleInRadians / 2));
    aim_yaw.Normalize();

    aim_p = currentTarget;
    aim_dp = Vector3Df(0, 0, 0);

    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);

    MixOrientation();
    q = mixQuaternion;
    w = mixAngSpeed;

    uav_p = uav_p - ref_p;
    Quaternion qze = aim_yaw.GetConjugate() * q;
    Vector3Df thetaze = 2 * qze.GetLogarithm();
    float zsign = 1;
    if (thetaze.GetNorm() >= 3.1416) {
        zsign = -1;
    }
    aim_yaw = zsign * aim_yaw;


    // myLaw->SetRejectionPercent(rejectionPercent);
    // myLaw->SetTarget(aim_p, aim_dp, aim_yaw);
    // myLaw->UpdateDynamics(uav_p, uav_dp, q, w);
    // myLaw->Update(GetTime());
}

void Drone::TargetFollowControl(){
    
}


void Drone::TestObserverSequence(void){
   

}