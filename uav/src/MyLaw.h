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


        // Flags
        bool isDisturbanceActive; // Flag for disturbance activation
        bool isDisturbanceRotActive; // Flag for disturbance rotational activation
        bool isKalmanActive;


        ///////////////////////////
        // LAYOUT GAINS AND PARAMS
        ///////////////////////////

        flair::filter::Pid *uX_custom, *uY_custom;
        flair::filter::PidThrust *uZ_custom;

        // Layout drone properties
        DoubleSpinBox *mass_layout;
                
        // Layout UDE Gain
        Vector3DSpinBox *omega_gains_trans;
        Vector3DSpinBox *omega_gains_rot;

        // Motor constant
        DoubleSpinBox *motorConst;
                
        ///////////////////////////
        // TARGET VARS
        ///////////////////////////

        Vector3Df p_d;    // Desire position

        ///////////////////////////
        // CONTROL INPUTS
        ///////////////////////////

        float Fu;
        Vector3Df Tauu;
        float motorK;

        ///////////////////////////
        // PERTURBATIONS
        ///////////////////////////

        // Perturbations
        Vector3Df perturbation_trans;
        Vector3Df perturbation_rot;

        Vector3Df rejectionPercent;
        Vector3Df rejectionRotPercent;
                
        // Custom time conditions
        bool firstUpdate;

        ///////////////////////////
        // TIME
        ///////////////////////////
                
        std::chrono::high_resolution_clock::time_point previous_chrono_time;

        ///////////////////////////
        // CSV HANDLERS
        ///////////////////////////

    
        std::ofstream errorsOutputFileCSV;
        std::string errorsFilePath;

            
        ///////////////////////////
        // DYNAMIC VARIABLES MATRIX
        ///////////////////////////

        Matrix *stateM, *dataexp, *input; // Description Matrix

                
        ///////////////////////////
        // CONSTRUCTOR
        ///////////////////////////

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
        void UpdateFrom(const io_data *data);
        void CalculateControl(float dt);
                
        ///////////////////////////
        // UPDATE DYNAMIC VARS
        /////////////////////////// 
                
        void UpdateDynamics(Vector3Df p, Vector3Df pd, Quaternion q,Vector3Df w);
        void Reset(void); // No esta bien implementada aun
                
        ///////////////////////////
        // CSV FUNcS
        /////////////////////////// 

        void SaveErrorsCSV(Vector3Df &ep, Quaternion &eq, float &dt);

    };
}
}

#endif //MYLAW_H
































