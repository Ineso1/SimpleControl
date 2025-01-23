#include "MyLaw.h"

namespace flair
{
namespace filter
{
	
MyLaw::MyLaw(const LayoutPosition* position, string name) {
       
    previous_chrono_time = std::chrono::high_resolution_clock::now();
    firstUpdate = true;
    isDisturbanceActive = false;
    isKalmanActive = false;
    rejectionPercent = Vector3Df(0,0,0);
    rejectionRotPercent = Vector3Df(0,0,0);

    /***********************
    WEIGHT LAYOUT
    ************************/

    perturbation_trans = Vector3Df(0,0,0);
    perturbation_rot = Vector3Df(0,0,0);

    GroupBox* customPID_groupbox = new GroupBox(position,name);

    uX_custom = new Pid(customPID_groupbox->At(1,0),"u_x_custom");
    uY_custom = new Pid(customPID_groupbox->At(1,1),"u_y_custom");
    uZ_custom = new PidThrust(customPID_groupbox->At(1,3), "u_z_custom");


    omega_gains_rot = new Vector3DSpinBox(customPID_groupbox->NewRow(),"omegaUDE_rot",0,100,0.01,3,Vector3Df(80.0,80.0,80.0));
    omega_gains_trans = new Vector3DSpinBox(customPID_groupbox->NewRow(),"omegaUDE_trans",0,100,0.01,3,Vector3Df(60.0,60.0,60.0));
    
    mass_layout = new DoubleSpinBox(customPID_groupbox->NewRow(),"Massa que mas aplauda",0.1,10,0.001,3,0.405);
    motorConst = new DoubleSpinBox(customPID_groupbox->NewRow(),"Motor const",0,20,0.0001,10,10);

    /***********************
    CSV write instances
    ************************/
    #ifdef SAVE_ERRORS_CSV
        errorsFilePath = ERRRORS_FILE_PATH_CSV;
        errorsOutputFileCSV.open(errorsFilePath, std::ios::trunc);
        errorsOutputFileCSV    << "ep_x,ep_y,ep_z,eq_w,eq_x,eq_y,eq_z,dt\n";
    #endif
}

MyLaw::~MyLaw(void) {
    if (errorsOutputFileCSV.is_open()) {
        errorsOutputFileCSV.close();
    }
}


/***********************
MY CONTROL CHIDO >:v
************************/
void MyLaw::UpdateFrom(const io_data *data) {
    if(firstUpdate){
        previous_chrono_time = std::chrono::high_resolution_clock::now();
    }
    
    // dt Calc
    float dt;
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> alt_dt = current_time - previous_chrono_time;
    dt = alt_dt.count();
    previous_chrono_time = current_time;



    firstUpdate = false;
}


void MyLaw::UpdateTranslationControl(Vector3Df& current_p, Vector3Df &current_dp, Quaternion current_q){
    Vector3Df pos_err = current_p - p_d;
    Vector3Df vel_err = current_dp - Vector3Df(0,0,0);

    Euler currentAngles;//in vrpn frame
    current_q.ToEuler(currentAngles);
    pos_err.Rotate(-currentAngles.yaw);
    vel_err.Rotate(-currentAngles.yaw);

    uX_custom->SetValues(pos_err.x, vel_err.x);
    uX_custom->Update(GetTime());

    uY_custom->SetValues(pos_err.y, vel_err.y);
    uY_custom->Update(GetTime());
}

void MyLaw::UpdateThrustControl(Vector3Df& current_p ,Vector3Df &aim_p, Vector3Df &current_dp, Vector3Df &aim_dp, Quaternion current_q){

}


void MyLaw::Reset(void) {
    p_d.x = 0;
    p_d.y = 0;
    p_d.z = 0;
}

void MyLaw::CalculateControl(float dt) {
    
    
}



void MyLaw::SaveErrorsCSV(Vector3Df &ep, Quaternion &eq, float &dt){
    if (errorsOutputFileCSV.is_open()) {
        errorsOutputFileCSV    << std::fixed << std::setprecision(6)
                                    << ep.x << "," << ep.y << "," << ep.z << ","
                                    << eq.q0 << "," << eq.q1 << "," << eq.q2 << "," << eq.q3 << "," << dt << "\n";
    } else {
        std::cerr << "Error opening Errors.csv\n";
    }
}

void MyLaw::UpdateDynamics(Vector3Df p, Vector3Df dp, Quaternion q,Vector3Df w){
    input->SetValue(0,0,q.q0);
    input->SetValue(1,0,q.q1);
    input->SetValue(2,0,q.q2);
    input->SetValue(3,0,q.q3);
    input->SetValue(8,0,w.x);
    input->SetValue(9,0,w.y);
    input->SetValue(10,0,w.z);
    input->SetValue(14,0,p.x);
    input->SetValue(15,0,p.y);
    input->SetValue(16,0,p.z);
    input->SetValue(17,0,dp.x);
    input->SetValue(18,0,dp.y);
    input->SetValue(19,0,dp.z);
};

void MyLaw::SetTarget(Vector3Df target_pos, Vector3Df target_vel, Quaternion target_yaw){
    p_d = target_pos;
}

void MyLaw::SetPerturbation(Vector3Df p_trans, Vector3Df p_rot){
    perturbation_trans = p_trans;
    perturbation_rot = p_rot;
}

void MyLaw::SetRejectionPercent(Vector3Df rejection){
    rejectionPercent = rejection;
}

} // end nasmespace filter
} // end namespace flair


