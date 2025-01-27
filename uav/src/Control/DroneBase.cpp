// DroneBase.cpp
#include "DroneBase.h"

DroneBase::DroneBase(TargetController *controller) : UavStateMachine(controller), vrpnLost(false), behaviourMode(BehaviourMode_t::Default) {
    uav = GetUav();
    
    customLawTab = new Tab(getFrameworkManager()->GetTabWidget(), "Custom Law");
    execLayout = new GridLayout(customLawTab->NewRow(), "Custom Law Layout");

    beahviourMode_layout = new ComboBox(GetButtonsLayout()->NewRow(),"Select behavior");
    beahviourMode_layout->AddItem("Position");
    beahviourMode_layout->AddItem("Trajectory"); 
    beahviourMode_layout->AddItem("Sequence"); 

    startTrajectory = new PushButton(GetButtonsLayout()->LastRowLastCol(), "Start Slected Behaiour");
    stopTrajectory = new PushButton(GetButtonsLayout()->LastRowLastCol(), "Stop");
    positionHold = new PushButton(GetButtonsLayout()->LastRowLastCol(), "Hold (not implemented yet)");

    positionBehaveBox = new GroupBox(GetButtonsLayout()->NewRow(),"Pos");
    targetPosition_layout = new Vector3DSpinBox(positionBehaveBox->NewRow(), "Target Pos", -5, 5, 0.0001, 6, Vector3Df(0.0, 0.0, 0.0));
    yawAngle_layout = new DoubleSpinBox(positionBehaveBox->NewRow(), "Yaw Euler", -180, 180, 0.0001, 10, 0);
    positionChange = new PushButton(positionBehaveBox->NewRow(), "Change Target");

    disturbanceEstimator = new GroupBox(GetButtonsLayout()->LastRowLastCol(),"DistEst");
    observerMode_layout = new ComboBox(disturbanceEstimator->NewRow(),"Select observer");
    observerMode_layout->AddItem("UDE");
    observerMode_layout->AddItem("Luenberger");
    observerMode_layout->AddItem("SuperTwisting");
    observerMode_layout->AddItem("SlidingMode");
    rejectionPercent_layout = new Vector3DSpinBox(disturbanceEstimator->NewRow(), "Rejection", 0, 1, 0.0001, 6, Vector3Df(0.8, 0.8, 1.0));
    rejectionModeState = new Label(disturbanceEstimator->NewRow(), "state");
    rejectionModeState->SetText("state: ----- off");
    Label *someSpace_1 = new Label(disturbanceEstimator->NewRow(), "space");
    rejectPerturbation = new PushButton(disturbanceEstimator->NewRow(), "Reject Disturbance");
    
    disturbanceRotEstimator = new GroupBox(GetButtonsLayout()->LastRowLastCol(),"DistRotEst");
    Label *someSpace_2_1 = new Label(disturbanceRotEstimator->NewRow(), "space");
    rejectionPercentRot_layout = new Vector3DSpinBox(disturbanceRotEstimator->NewRow(), "Rejection Rot", 0, 1, 0.0001, 6, Vector3Df(0.8, 0.8, 1.0));
    rejectionRotModeState = new Label(disturbanceRotEstimator->NewRow(), "state");
    rejectionRotModeState->SetText("state: ----- off");
    Label *someSpace_2 = new Label(disturbanceRotEstimator->NewRow(), "space");
    rejectRotPerturbation = new PushButton(disturbanceRotEstimator->NewRow(), "Reject Disturbance");
    

    disturbanceSim = new GroupBox(GetButtonsLayout()->LastRowLastCol(),"DistSim");
    perturbation_layout = new Vector3DSpinBox(disturbanceSim->NewRow(), "Disturbance", -10, 10, 0.0001, 6, Vector3Df(0.0, 0.0, 0.0));
    Label *someSpace_3 = new Label(disturbanceEstimator->NewRow(), "space");
    disturbanceModeState = new Label(disturbanceSim->NewRow(), "state");
    disturbanceModeState->SetText("state: ----- off");
    togglePerturbation = new PushButton(disturbanceSim->NewRow(), "Toggle Disturbance");

    kalmanActivation = new GroupBox(GetButtonsLayout()->LastRowLastCol(),"Kalman Filter");
    kalmanActivationState = new Label(kalmanActivation->NewRow(), "space");
    kalmanActivationState->SetText("state: ----- off");
    toggleKalman = new PushButton(kalmanActivation->NewRow(), "Toggle Kalman Activation");


    VrpnClient* vrpnclient = new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(), 80, uav->GetDefaultVrpnConnectionType());
    if (vrpnclient->ConnectionType() == VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(), static_cast<uint8_t>(0));
    } else if (vrpnclient->ConnectionType() == VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    } else if (vrpnclient->ConnectionType() == VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    }
    if (uav->GetType() == "mamboedu") {
        SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    vrpnclient->Start();
}

DroneBase::~DroneBase() {
}

void DroneBase::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch (event) {
    case Event_t::TakingOff:
        behaviourMode = BehaviourMode_t::Default;
        vrpnLost = false;
        break;
    case Event_t::EnteringControlLoop:
        behaviourMode = BehaviourMode_t::Trajectory;
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode = BehaviourMode_t::Default;
        break;
    }
}

void DroneBase::ExtraSecurityCheck() {
    if (!vrpnLost && (behaviourMode == BehaviourMode_t::Trajectory || behaviourMode == BehaviourMode_t::PositionHold)) {
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, UAV lost\n");
            vrpnLost = true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void DroneBase::ExtraCheckPushButton() {
    if (startTrajectory->Clicked()) {
        StartTrajectory();
    }
    if (stopTrajectory->Clicked()) {
        StopTrajectory();
    }
    if (positionHold->Clicked()) {
        PositionHold();
    }
    if (positionChange->Clicked()) {
        PositionChange();
    }
    if (togglePerturbation->Clicked()) {
        HandleDisturbanceToggle();
    }
    if (rejectPerturbation->Clicked())
    {
        RejectDisturbance();
    }
    if (rejectRotPerturbation->Clicked())
    {
        RejectRotDisturbance();
    }
    if (toggleKalman->Clicked())
    {
        ApplyKalman();
    }
}

void DroneBase::ExtraCheckJoystick() {
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        StartTrajectory();
    }

    //R1 and Cross
    if(GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
        StopTrajectory();
    }
    
    //R1 and Square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        PositionHold();
    }
}


void DroneBase::StopTrajectory(void) {
    if( behaviourMode!=BehaviourMode_t::Default) {
        SetThrustMode(ThrustMode_t::Default);
        SetTorqueMode(TorqueMode_t::Default);
        Thread::Warn("DroneBase: not in Trajectory mode\n");
        return;
    }
    Thread::Info("UDEdrone: StopTrajectory\n");
}

void DroneBase::PositionHold(void) {
    if( behaviourMode!=BehaviourMode_t::Default) {
        SetThrustMode(ThrustMode_t::Default);
        SetTorqueMode(TorqueMode_t::Default);
        Thread::Warn("DroneBase: not in Trajectory mode\n");
        return;
    }
    Thread::Info("UDEdrone: StopTrajectory\n");
}

void DroneBase::StartTrajectory() {
    if( behaviourMode == BehaviourMode_t::Trajectory) {
        Thread::Warn("UDEdrone: already in this mode\n");
    }
    if (SetTorqueMode(TorqueMode_t::Custom)) {
        Thread::Info("esta vaina: start \n");
    } else {
        Thread::Warn("esta vaina: could not start otra vez \n");
        return;
    }
    if (SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Info("UDEdrone: Custom Thrust Mode Set\n");
    } else {
        Thread::Warn("UDEdrone: Failed to set Custom Thrust Mode\n");
    }
    ApplyControl();
}

void DroneBase::PositionChange(void) {
    std::cout << "Need to override this" << std::endl;
}

void DroneBase::HandleDisturbanceToggle() {
    std::cout << "Need to override this" << std::endl;
}

void DroneBase::RejectDisturbance() {
    std::cout << "Need to override this" << std::endl;
}

void DroneBase::RejectRotDisturbance() {
    std::cout << "Need to override this" << std::endl;
}

void DroneBase::ApplyControl(void){
    std::cout << "Need to override this" << std::endl;
}

void DroneBase::ApplyKalman(void){
    std::cout << "Need to override this" << std::endl;
}