#ifndef Law
// Law.h

#include <Matrix.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <SpinBox.h>
#include <GroupBox.h>
#include <Quaternion.h>
#include <ControlLaw.h>
#include <string>

namespace flair {
    namespace gui {
        class LayoutPosition;
    }
}
namespace flair {
namespace filter {
    class Law : public ControlLaw {
        public:
            flair::gui::DoubleSpinBox *T;
            flair::gui::Vector3DSpinBox *kpatt;
            flair::gui::Vector3DSpinBox *kdatt;
            flair::gui::DoubleSpinBox *satAtt;
            flair::gui::DoubleSpinBox *PosRefRate;

            flair::gui::Vector3DSpinBox *kppos;
            flair::gui::Vector3DSpinBox *omg;
            flair::gui::Vector3DSpinBox *kdpos;
			flair::gui::DoubleSpinBox *satPos;
            flair::gui::DoubleSpinBox *satPosForce;
            flair::gui::DoubleSpinBox *mg;
            flair::gui::DoubleSpinBox *CustomDeltaT;
            flair::core::Vector3Df posd;
            flair::core::Vector3Df Eps,Epsd,west,omega;
            flair::core::Vector3Df Fu;
            float previous_time;
            bool first_update;
            float timeT;
            bool first=true;
            // flair::core::MatrixDescriptor desc;
            flair::core::Matrix *stateM;
            
            ~Law(void);
            Law(const gui::LayoutPosition* position,std::string name);
            void UseDefaultPlot(const gui::LayoutPosition* position);
            void Reset(void);
            void UpdateFrom(const core::io_data *data);
            void SetValues(core::Quaternion q,core::Quaternion qd,core::Vector3Df w,core::Vector3Df pos, core::Vector3Df posd,core::Vector3Df vel,core::Vector3Df veld);
        private:
    };
}
}

#endif // CIRCLEFOLLOWER