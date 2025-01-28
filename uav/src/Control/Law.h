#ifndef LAW
#define LAW

#include <Matrix.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <SpinBox.h>
#include <GroupBox.h>
#include <GridLayout.h>
#include <Quaternion.h>
#include <ControlLaw.h>
#include <string>
#include <chrono>
#include "../Observer/UDE/UDE.h"
#include "../Observer/SlidingMode/SlidingMode.h"
#include <Tab.h>

namespace flair {
    namespace gui {
        class LayoutPosition;
    }
}
namespace flair {
namespace filter {
    class Law : public ControlLaw {
        public:
            
            enum class ObserverMode_t { UDE, Luenberger, SuperTwist, SlidingMode };
            ObserverMode_t observerMode;
            bool isDisturbanceActive; // Flag for disturbance activation
            bool isDisturbanceRotActive; // Flag for disturbance rotational activation
            bool isKalmanActive;

            std::chrono::high_resolution_clock::time_point previous_chrono_time;

            flair::core::Vector3Df u_thrust;
            flair::core::Vector3Df u_torque;

            float g;
            float mass;

            flair::core::Vector3Df w_estimation_trans;
            flair::core::Vector3Df w_estimation_rot;
            flair::core::Vector3Df rejectionPercent;
            flair::core::Vector3Df rejectionRotPercent;

            flair::core::Vector3Df perturbation_trans;

            flair::gui::GridLayout *controlLayout;
            flair::gui::GridLayout *paramsLayout;

            flair::gui::GroupBox *control_groupbox_att;
            flair::gui::GroupBox *control_groupbox_trans;
            
            flair::gui::Vector3DSpinBox *kpatt;
            flair::gui::Vector3DSpinBox *kdatt;
            flair::gui::DoubleSpinBox *satAtt;

            flair::gui::Vector3DSpinBox *kppos;
            flair::gui::Vector3DSpinBox *kdpos;
			flair::gui::DoubleSpinBox *satPos;
            flair::gui::DoubleSpinBox *satPosForce;

            flair::gui::DoubleSpinBox *mg;
            flair::gui::DoubleSpinBox *CustomDeltaT;


            flair::core::Vector3Df p_d;    // Desire position
            flair::core::Vector3Df dp_d;   // Desire velocity
            flair::core::Vector3Df Fu;

            Observer::UDE ude_obs;
            flair::gui::Vector3DSpinBox *omegaGainsTrans;
            flair::gui::Vector3DSpinBox *omegaGainsRot;

            Observer::SlidingMode slidingMode_obs;
            flair::gui::DoubleSpinBox *smUpperBoundTrans;
            flair::gui::DoubleSpinBox *smUpperBoundRot;
            flair::gui::DoubleSpinBox *smFilterTrans;
            flair::gui::DoubleSpinBox *smFilterRot;

            float previous_time;
            bool first_update;
            
            flair::core::MatrixDescriptor *desc;
            flair::core::Matrix *stateM;
            
            ~Law(void);
            Law(const gui::LayoutPosition* position,std::string name);
            void UseDefaultPlot(const gui::LayoutPosition* position);
            void Reset(void);
            void UpdateFrom(const core::io_data *data);
            void SetValues(core::Quaternion q,core::Quaternion qd,core::Vector3Df w,core::Vector3Df p, core::Vector3Df p_d,core::Vector3Df dp,core::Vector3Df dp_d);
        
            void SetTarget(core::Vector3Df, core::Vector3Df, core::Quaternion);
            void SetRejectionPercent(core::Vector3Df, core::Vector3Df);
            void SetPerturbation(core::Vector3Df, core::Vector3Df);

        private:
    };
}
}

#endif // CIRCLEFOLLOWER