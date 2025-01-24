#ifndef MYLAW_H
#define MYLAW_H

#include <Matrix.h>
#include <Layout.h>
#include <Tab.h>
#include <GridLayout.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3D.h>
#include <Vector3DSpinBox.h>
#include <SpinBox.h>
#include <GroupBox.h>
#include <Quaternion.h>
#include <Euler.h>
#include <ControlLaw.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <Pid.h>
#include <PidThrust.h>
#include <IODevice.h>
#include "../Observer/UDE/UDE.h"
#include "../Observer/Luenberger/Luenberger.h"
#include "../Observer/SlidingMode/SlidingMode.h"
#include "../Observer/SuperTwist/SuperTwist.h"
#include <Eigen/Dense>

using namespace std;
using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
    namespace gui {
        class LayoutPosition;
    }
}

namespace flair {
namespace filter {
    class MyLaw {
        public:

        enum class ObserverMode_t { UDE, Luenberger, SuperTwist, SlidingMode };
        ObserverMode_t observerMode;

        bool isDisturbanceActive; // Flag for disturbance activation
        bool isDisturbanceRotActive; // Flag for disturbance rotational activation
        bool isKalmanActive;

        flair::filter::Pid *uX_custom, *uY_custom;
        flair::filter::PidThrust *uZ_custom;

        // Layout drone properties
        DoubleSpinBox *mass_layout;
                
        Vector3DSpinBox *omega_gains_trans;
        Vector3DSpinBox *omega_gains_rot;
        DoubleSpinBox *motorConst;

        Vector3Df p_d;    // Desire position

        float motorK;
        float g;
        float mass;
        Vector3Df u_thrust;
        Vector3Df u_torque;

        Vector3Df w_estimation_trans;
        Vector3Df w_estimation_rot;

        Vector3Df perturbation_trans;
        Vector3Df perturbation_rot;
        Vector3Df rejectionPercent;
        Vector3Df rejectionRotPercent;
                
        bool firstUpdate;

        std::chrono::high_resolution_clock::time_point previous_chrono_time;

        std::ofstream errorsOutputFileCSV;
        std::string errorsFilePath;
        std::ofstream translationOutputFileCSV;
        std::string translationFilePath; 
        std::ofstream rotationOutputFileCSV;
        std::string rotationFilePath;

        Matrix *stateM, *dataexp, *input; // Description Matrix

        Observer::UDE ude;
        Observer::Luenberger luenberger;
        Observer::SlidingMode slidingMode;
        Observer::SuperTwist superTwist;


        ~MyLaw(void);
        MyLaw(const LayoutPosition* position,std::string name);

        ///////////////////////////
        // SETTERS
        ///////////////////////////

        void SetTarget(Vector3Df, Vector3Df, Quaternion);
        void SetPerturbation(Vector3Df, Vector3Df);
        void SetRejectionPercent(Vector3Df);

        ///////////////////////////
        // MA CONTROL ALGO
        /////////////////////////// 
        void UpdateTranslationControl(Vector3Df& current_p , Vector3Df &current_dp, Quaternion &current_q);
        void UpdateThrustControl(Vector3Df& current_p , Vector3Df &current_dp);
        void CalculateControl(Vector3Df& current_p , Vector3Df &current_dp, Quaternion &current_q, Vector3Df &current_omega);
                
        ///////////////////////////
        // UPDATE DYNAMIC VARS
        /////////////////////////// 
                
        void UpdateDynamics(Vector3Df p, Vector3Df pd, Quaternion q,Vector3Df w);
        void Reset(void); // No esta bien implementada aun
                
        ///////////////////////////
        // CSV FUNcS
        /////////////////////////// 

        void SaveErrorsCSV(Vector3Df &ep, Quaternion &eq, float &dt);
        void SaveStateCSV(Vector3Df &p, Vector3Df &dp,Vector3Df &ddp, Vector3Df &domega, Vector3Df &omega, Quaternion &dq, Quaternion &q, float &dt);
    };
}
}

#endif //MYLAW_H
































