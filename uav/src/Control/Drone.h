// Drone.h
#ifndef DRONE_H
#define DRONE_H

#include "DroneBase.h"
#include "Flags.h"
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
#include <chrono>
#include "Law.h"
#include <iostream>
#include "Trayectory.h"
#include "SoftTrajectoryGenerator.h"

class Drone : public DroneBase {
public:
    Drone(TargetController *controller);
    virtual ~Drone();

protected:
    enum class AlgorithmBehaviourMode_t {PositionPoint, TrajectoryFollow};
    AlgorithmBehaviourMode_t algorithmBehaviourMode;
    std::chrono::high_resolution_clock::time_point previous_chrono_time_sequence;
    float sequenceTime;
    bool sequenceFirstTime;

    flair::core::Vector3Df sequenceInitPos;

    float refAltitude;
    float refVerticalVelocity;
    float yawHold;

    // Perturbation Toggle
    bool perturbation;
    bool kalman;

    // Feedback objects
    flair::core::Vector3Df vrpnPosition;
    flair::core::Quaternion vrpnQuaternion;
    flair::core::Vector3Df currentTarget;
    float yawAngle;
    Quaternion initQuaternion;

    // Control Law
    Law *myLaw;
    Trayectory trayectory_circle;
    SoftTrajectoryGenerator softTrajectory;

    // Buttons handlers
    void PositionChange() override;
    void HandleDisturbanceToggle(void) override;
    void ApplyKalman(void) override;
    void RejectDisturbance(void) override;
    void RejectRotDisturbance(void) override;
    void ResetSequence(void) override;

    // Control behave
    void ApplyControl(void) override;

    // Correction Functions
    void MixOrientation(void);

    // Control inputs
    float ComputeCustomThrust() override;
    void ComputeCustomTorques(flair::core::Euler &torques);
    const flair::core::AhrsData *GetOrientation(void) const override;
    void AltitudeValues(float &z,float &dz) const override;

    // Control behave algorithm functions
    void PositionControl(void);
    void TargetFollowControl(void);
    void TestObserverSequence(void);


private:
    Quaternion mixQuaternion;
    Vector3Df mixAngSpeed;
    Quaternion qI;
    bool first_up = true;
    flair::core::AhrsData *customReferenceOrientation,*customOrientation;
};

#endif // DRONE_H
