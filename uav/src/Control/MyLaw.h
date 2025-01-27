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
#include <iomanip>
#include <Pid.h>
#include <PidThrust.h>

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
    class MyLaw 
    : public ControlLaw
    {
        public:

        enum class ObserverMode_t { UDE, Luenberger, SuperTwist, SlidingMode };
        ObserverMode_t observerMode;

        bool isDisturbanceActive; // Flag for disturbance activation
        bool isDisturbanceRotActive; // Flag for disturbance rotational activation
        bool isKalmanActive;

        GridLayout *controlLayout;
        GridLayout *paramsLayout;

        GroupBox *control_groupbox_att;
        Vector3DSpinBox *kpatt;
        Vector3DSpinBox *kdatt;
        DoubleSpinBox *satAtt;

        GroupBox *control_groupbox_trans;
        Vector3DSpinBox *kppos;
        Vector3DSpinBox *kdpos;
        DoubleSpinBox *satPos;
        DoubleSpinBox *satPosForce;
        DoubleSpinBox *mg;

        // Layout drone properties
        DoubleSpinBox *mass_layout;
                
        Vector3DSpinBox *omega_gains_trans;
        Vector3DSpinBox *omega_gains_rot;
        DoubleSpinBox *motorConst;

        Vector3Df p_d;    // Desire position
        Vector3Df dp_d;    // Desire velocity

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

        float previous_time;

        std::ofstream errorsOutputFileCSV;
        std::string errorsFilePath;
        std::ofstream translationOutputFileCSV;
        std::string translationFilePath; 
        std::ofstream rotationOutputFileCSV;
        std::string rotationFilePath;

        Matrix *stateM, *dataexp; // Description Matrix

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
        void UpdateTranslationControl();
        void UpdateThrustControl();
        void CalculateControl(Vector3Df& current_p , Vector3Df &current_dp, Quaternion &current_q, Vector3Df &current_omega);
                
        ///////////////////////////
        // UPDATE DYNAMIC VARS
        /////////////////////////// 
                
        void SetValues(Vector3Df& p, Vector3Df& pd, Quaternion& q,Vector3Df& w);
        void UpdateFrom(const core::io_data *data);
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
































