// DroneBase.h
#ifndef DRONEBASE_H
#define DRONEBASE_H 

#include <UavStateMachine.h>
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <PidThrust.h>
#include <Vector3DSpinBox.h>
#include <DoubleSpinBox.h>
#include <ComboBox.h>
#include <GroupBox.h>
#include <Label.h>
#include <iostream>

using namespace std;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::core;

class DroneBase : public UavStateMachine {
public:
    Uav* uav;
    bool vrpnLost;
    DroneBase(TargetController *controller);
    virtual ~DroneBase();

protected:
    enum class BehaviourMode_t { Default, PositionHold, Trajectory, Sequence };
    // enum class BehaviourMode_t { Default, PositionControl, TrajectoryFollower };
    BehaviourMode_t behaviourMode;

    // UI Buttons
    Tab *customLawTab;
    GroupBox* positionBehaveBox;
    GroupBox* disturbanceEstimator;
    GroupBox* disturbanceRotEstimator;
    GroupBox *disturbanceSim;
    GroupBox *kalmanActivation;

    GridLayout* execLayout;
    PushButton *startTrajectory;
    PushButton *stopTrajectory; 
    PushButton *positionHold;
    PushButton *positionChange;
    PushButton *togglePerturbation;
    PushButton *toggleKalman;
    Vector3DSpinBox *targetPosition_layout;
    DoubleSpinBox *yawAngle_layout;
    Vector3DSpinBox *rejectionPercent_layout;
    Vector3DSpinBox *rejectionPercentRot_layout;
    PushButton *rejectPerturbation;
    PushButton *rejectRotPerturbation;
    Vector3DSpinBox *perturbation_layout;

    ComboBox *beahviourMode_layout;
    ComboBox *observerMode_layout;

    Label *disturbanceModeState;
    Label *rejectionModeState;
    Label *rejectionRotModeState;
    Label *kalmanActivationState;

    MetaVrpnObject *uavVrpn;

    // Methods
    void StopTrajectory(void);
    void StartTrajectory(void);
    void PositionHold(void);

    virtual void HandleDisturbanceToggle(void);
    virtual void ApplyControl(void);
    virtual void PositionChange(void);
    virtual void RejectDisturbance(void);
    virtual void RejectRotDisturbance(void);
    virtual void ApplyKalman(void);

    // State Machine Functions
    void SignalEvent(Event_t event) override;
    void ExtraSecurityCheck(void) override;
    void ExtraCheckPushButton(void) override;
    void ExtraCheckJoystick(void) override;

};

#endif // DRONEBASE_H