// Drone.cpp
#include "Drone.h"

Drone::Drone(TargetController *controller) : DroneBase(controller) {
    
    //Behave
    algorithmBehaviourMode = AlgorithmBehaviourMode_t::PositionPoint;
    sequenceFirstTime = true;
    currentTarget = flair::core::Vector3Df(0,0,-1);

    // Toggle variables
    kalman = false;
    perturbation = false;

    float diameter = 8.0;
    float fixedZ = -1.0;
    float resolution = 0.002;
    trayectory_circle = Trayectory(diameter,fixedZ,resolution);

    // Law instance
    myLaw = new Law(execLayout->At(1, 0), "MyLaw");

    // Set Zero Orientation 
    initQuaternion =  GetCurrentQuaternion();
    customReferenceOrientation= new AhrsData(this,"reference");
    customOrientation=new AhrsData(this,"orientation");

    // Set current position target
    flair::core::Vector3Df uav_p;
    uavVrpn->GetPosition(uav_p);
    currentTarget = Vector3Df(0,0,-1);

    refAltitude = 1;
    refVerticalVelocity = 0;
    yawHold=vrpnQuaternion.ToEuler().yaw;

    myLaw->SetTarget(currentTarget, Vector3Df(0,0,0), Quaternion(1,0,0,0));


    /*
    softTrajectory.addWaypoint(Eigen::Vector3f(0, 0, -1), 0);
    softTrajectory.addWaypoint(Eigen::Vector3f(0, 0, -1), 5);
    softTrajectory.addWaypoint(Eigen::Vector3f(0.5, 0, -1), 10);
    softTrajectory.addWaypoint(Eigen::Vector3f(0.5, -0.5, -1), 15);
    softTrajectory.addWaypoint(Eigen::Vector3f(-1.5, -0.5, -1), 20);
    softTrajectory.addWaypoint(Eigen::Vector3f(-1.5, 0, -1), 25);
    softTrajectory.addWaypoint(Eigen::Vector3f(0, 0, -1), 30);
    */

}

Drone::~Drone() {
    delete myLaw;
}

/***************************************** 
 * Buttons Handlers
*****************************************/

void Drone::PositionChange(void) {
    if(algorithmBehaviourMode == AlgorithmBehaviourMode_t::PositionPoint){
        currentTarget = flair::core::Vector3Df(targetPosition_layout->Value().x, targetPosition_layout->Value().y, -targetPosition_layout->Value().z);
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

void Drone::ResetSequence() {
    sequenceFirstTime = true;
}

/***************************************** 
 * Control behave
*****************************************/

void Drone::ApplyControl() {
    switch(observerMode_layout->CurrentIndex()){
        case 0:
            myLaw->observerMode = Law::ObserverMode_t::UDE;
            break;
        case 1:
            myLaw->observerMode = Law::ObserverMode_t::Luenberger;
            break;
        case 2:
            myLaw->observerMode = Law::ObserverMode_t::SuperTwist;
            break;
        case 3:
            myLaw->observerMode = Law::ObserverMode_t::SlidingMode;
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
    return myLaw->Output(3);
}

void Drone::ComputeCustomTorques(Euler &torques) {
    ApplyControl();
    torques.roll = myLaw->Output(0);
    torques.pitch = myLaw->Output(1);
    torques.yaw  = myLaw->Output(2);
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


void Drone::AltitudeValues(float &z,float &dz) const{
    Vector3Df p,dp;

    uavVrpn->GetPosition(p);
    uavVrpn->GetSpeed(dp);
    //z and dz must be in uav's frame
    z=-p.z;
    dz=-dp.z;
}

/***************************************** 
 * Control behave algorithm functions
*****************************************/

void Drone::PositionControl(){
    Vector3Df uav_p, uav_dp; 
    Vector3Df aim_p, aim_dp;
    Vector3Df rejectionPercent = flair::core::Vector3Df(rejectionPercent_layout->Value().x, rejectionPercent_layout->Value().y, rejectionPercent_layout->Value().z);
    Vector3Df rejectionRotPercent = flair::core::Vector3Df(rejectionPercentRot_layout->Value().x, rejectionPercentRot_layout->Value().y, rejectionPercentRot_layout->Value().z);

    Quaternion q;
    Vector3Df w;
    Quaternion qz(1,0,0,0);
    float yawref = 0;

    GetOrientation()->GetQuaternionAndAngularRates(q, w);

    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);

    aim_p = currentTarget;
    aim_dp = Vector3Df(0, 0, 0);

    myLaw->SetRejectionPercent(rejectionPercent, rejectionRotPercent);
    myLaw->SetTarget(aim_p, aim_dp, qz);


    
    
    
    qz = Quaternion(cos(yawref/2),0,0,sin(yawref/2));
    qz.Normalize();
    Quaternion qze = qz.GetConjugate()*q;
    Vector3Df thetaze = 2*qze.GetLogarithm();
    float zsign = 1;
    if (thetaze.GetNorm()>=3.1416){
        zsign = -1;
    }
    qz = zsign*qz;

    //std::cout<<"d p:\t"<<aim_p.x<<"\t"<<aim_p.y<<"\t"<<aim_p.z<<std::endl;


    myLaw->SetValues(q,qz,w,uav_p,aim_p,uav_dp,aim_dp);
    
    myLaw->Update(GetTime());
}




void Drone::TargetFollowControl(){
    Vector3Df uav_p, uav_dp; 
    Vector3Df aim_p, aim_dp;
    Vector3Df rejectionPercent = flair::core::Vector3Df(rejectionPercent_layout->Value().x, rejectionPercent_layout->Value().y, rejectionPercent_layout->Value().z);
    Vector3Df rejectionRotPercent = flair::core::Vector3Df(rejectionPercentRot_layout->Value().x, rejectionPercentRot_layout->Value().y, rejectionPercentRot_layout->Value().z);

    Quaternion q;
    Vector3Df w;
    Quaternion qz(1,0,0,0);
    float yawref = 0;

    GetOrientation()->GetQuaternionAndAngularRates(q, w);

    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);

    aim_p = trayectory_circle.getNextPoint();;
    aim_dp = Vector3Df(0, 0, 0);

    myLaw->SetRejectionPercent(rejectionPercent, rejectionRotPercent);
    myLaw->SetTarget(aim_p, aim_dp, qz);


    
    
    
    qz = Quaternion(cos(yawref/2),0,0,sin(yawref/2));
    qz.Normalize();
    Quaternion qze = qz.GetConjugate()*q;
    Vector3Df thetaze = 2*qze.GetLogarithm();
    float zsign = 1;
    if (thetaze.GetNorm()>=3.1416){
        zsign = -1;
    }
    qz = zsign*qz;

    //std::cout<<"d p:\t"<<aim_p.x<<"\t"<<aim_p.y<<"\t"<<aim_p.z<<std::endl;


    myLaw->SetValues(q,qz,w,uav_p,aim_p,uav_dp,aim_dp);
    
    myLaw->Update(GetTime());
}

void Drone::TestObserverSequence(void){
    Vector3Df uav_p, uav_dp; 
    Vector3Df aim_p, aim_dp;
    Vector3Df rejectionPercent = flair::core::Vector3Df(rejectionPercent_layout->Value().x, rejectionPercent_layout->Value().y, rejectionPercent_layout->Value().z);
    Vector3Df rejectionRotPercent = flair::core::Vector3Df(rejectionPercentRot_layout->Value().x, rejectionPercentRot_layout->Value().y, rejectionPercentRot_layout->Value().z);

    Quaternion q;
    Vector3Df w;
    Quaternion qz(1,0,0,0);
    float yawref = 0;

    GetOrientation()->GetQuaternionAndAngularRates(q, w);

    uavVrpn->GetPosition(uav_p);
    uavVrpn->GetSpeed(uav_dp);
    

    if(sequenceFirstTime){
        Vector3Df target_pos;
        Vector3Df real_target_pos;

        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            
            Land();
        }
        else
        {
            targetVrpn->GetPosition(target_pos);
        }
        
        

        real_target_pos.x = target_pos.x + chargePos->Value().x;
        real_target_pos.y = target_pos.y + chargePos->Value().y;
        real_target_pos.z = - target_pos.z - chargePos->Value().z;


        softTrajectory.addWaypoint(Eigen::Vector3f(0, 0, -1), 0);
        softTrajectory.addWaypoint(Eigen::Vector3f(0, 0, -1), 5);

        softTrajectory.addWaypoint(Eigen::Vector3f(real_target_pos.x, real_target_pos.y, real_target_pos.z - chargeHeight->Value()), 10);

        softTrajectory.addWaypoint(Eigen::Vector3f(real_target_pos.x, real_target_pos.y, real_target_pos.z), 15);

        softTrajectory.addWaypoint(Eigen::Vector3f(real_target_pos.x, real_target_pos.y, real_target_pos.z - chargeHeight->Value()), 20);

        softTrajectory.addWaypoint(Eigen::Vector3f(0, 0, -2), 25);
        softTrajectory.addWaypoint(Eigen::Vector3f(0, 0, -1), 30);

        softTrajectory.generateTrajectories();
        previous_chrono_time_sequence = std::chrono::high_resolution_clock::now();
        sequenceTime = 0;
        sequenceFirstTime = false;
    }

    float dt;
    auto current_time_sequence = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> alt_dt = current_time_sequence - previous_chrono_time_sequence;
    previous_chrono_time_sequence = current_time_sequence;
    dt = alt_dt.count();
    sequenceTime += dt;


    Eigen::Vector3f pos, vel;
    softTrajectory.getNextState(dt, pos, vel);

    aim_p = flair::core::Vector3Df(pos.x(), pos.y(), pos.z());
    aim_dp = flair::core::Vector3Df(vel.x(), vel.y(), vel.z());

    myLaw->SetRejectionPercent(rejectionPercent, rejectionRotPercent);
    myLaw->SetTarget(aim_p, aim_dp, qz);
    
    qz = Quaternion(cos(yawref/2),0,0,sin(yawref/2));
    qz.Normalize();
    Quaternion qze = qz.GetConjugate()*q;
    Vector3Df thetaze = 2*qze.GetLogarithm();
    float zsign = 1;
    if (thetaze.GetNorm()>=3.1416){
        zsign = -1;
    }
    qz = zsign*qz;

    myLaw->SetValues(q,qz,w,uav_p,aim_p,uav_dp,aim_dp);
    
    myLaw->Update(GetTime());

}