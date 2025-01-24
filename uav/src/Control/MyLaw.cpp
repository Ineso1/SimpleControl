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

    Vector3Df w_estimation_trans(0,0,0);
    Vector3Df w_estimation_rot(0,0,0);

    Vector3Df u_thrust(0,0,0);
    Vector3Df u_torque(0,0,0);
    observerMode = MyLaw::ObserverMode_t::UDE;
    mass = 0.445;
    g = 9.81;

    /************************
    Logs???
    ************************/
    input = new Matrix(position->getLayout(),23,1,floatType,name);
    MatrixDescriptor* desc = new MatrixDescriptor(23,1);
    desc->SetElementName(0,0,"q0");
    desc->SetElementName(1,0,"q1");
    desc->SetElementName(2,0,"q2");
    desc->SetElementName(3,0,"q3");
    desc->SetElementName(4,0,"wx");
    desc->SetElementName(5,0,"wy");
    desc->SetElementName(6,0,"wz");
    desc->SetElementName(7,0,"px");
    desc->SetElementName(8,0,"py");
    desc->SetElementName(9,0,"pz");
    desc->SetElementName(10,0,"u_roll");
    desc->SetElementName(11,0,"u_pitch");
    desc->SetElementName(12,0,"u_yaw");
    desc->SetElementName(13,0,"thrust");
    desc->SetElementName(14,0,"ecx");
    desc->SetElementName(15,0,"ecy");
    desc->SetElementName(16,0,"ecz");
    desc->SetElementName(17,0,"udeTx");
    desc->SetElementName(18,0,"udeTy");
    desc->SetElementName(19,0,"udeTz");
    desc->SetElementName(20,0,"udeRx");
    desc->SetElementName(21,0,"udeRy");
    desc->SetElementName(22,0,"udeRz");

    dataexp = new Matrix(position->getLayout(),desc,floatType,name);
    delete desc;
    // AddDataToLog(dataexp);


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

    

    #ifdef SAVE_ERRORS_CSV
        errorsFilePath = ERRRORS_FILE_PATH_CSV;
        errorsOutputFileCSV.open(errorsFilePath, std::ios::trunc);
        errorsOutputFileCSV    << "ep_x,ep_y,ep_z,eq_w,eq_x,eq_y,eq_z,dt\n";
    #endif
    #ifdef SAVE_REAL_STATE_SPACE_CSV
        translationFilePath = TRANSLATION_FILE_PATH_CSV;
        rotationFilePath = ROTATION_FILE_PATH_CSV;
        translationOutputFileCSV.open(translationFilePath, std::ios::trunc);
        rotationOutputFileCSV.open(rotationFilePath, std::ios::trunc);
        translationOutputFileCSV    << "p_x,p_y,p_z,dp_x,dp_y,dp_z,ddp_x,ddp_y,ddp_z,dt\n";
        rotationOutputFileCSV       << "domega_x,domega_y,domega_z,omega_x,omega_y,omega_z,"
                                    << "dq_w,dq_x,dq_y,dq_z,q_w,q_x,q_y,q_z,dt\n";
    #endif
}

MyLaw::~MyLaw(void) {
    if (errorsOutputFileCSV.is_open()) {
        errorsOutputFileCSV.close();
    }
}

void MyLaw::UpdateTranslationControl(Vector3Df& current_p, Vector3Df &current_dp, Quaternion &current_q){
    Vector3Df pos_err = current_p - p_d;
    std::cout<<"errZ "<< current_p.z << std::endl;
    Vector3Df vel_err = current_dp - Vector3Df(0,0,0);
    pos_err.Rotate(-current_q);
    vel_err.Rotate(-current_q);
    uX_custom->SetValues(pos_err.x, vel_err.x);
    uX_custom->Update(GetTime());
    uY_custom->SetValues(pos_err.y, vel_err.y);
    uY_custom->Update(GetTime());
}

void MyLaw::UpdateThrustControl(Vector3Df& current_p , Vector3Df &current_dp){
    Vector3Df pos_err = -current_p - p_d;
    Vector3Df vel_err = -current_dp - Vector3Df(0,0,0);
    uZ_custom->SetValues(pos_err.z, vel_err.z);
    uZ_custom->Update(GetTime());
}

void MyLaw::Reset(void) {
    p_d.x = 0;
    p_d.y = 0;
    p_d.z = 0;
}

void MyLaw::CalculateControl(Vector3Df& current_p , Vector3Df &current_dp, Quaternion &current_q, Vector3Df &current_omega) {
    Eigen::Vector3f w_estimation_trans_eig;
    Eigen::Vector3f w_estimation_rot_eig;
    mass = mass_layout->Value();
    
    if(firstUpdate){
        previous_chrono_time = std::chrono::high_resolution_clock::now();
    }
    float dt;
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> alt_dt = current_time - previous_chrono_time;
    dt = alt_dt.count();
    previous_chrono_time = current_time;
    if(firstUpdate){
        dt = 0;
    }
    std::cout<<dt<<std::endl;


    float thrust = uZ_custom->Output();
    u_thrust = Vector3Df(0,0,thrust);

    if(observerMode == MyLaw::ObserverMode_t::UDE){
        ude.u_thrust = Eigen::Vector3f(u_thrust.x, u_thrust.y, u_thrust.z);
        ude.u_torque = Eigen::Vector3f(u_torque.x, u_torque.y, u_torque.z);
        w_estimation_trans_eig = ude.EstimateDisturbance_trans(Eigen::Vector3f(current_p.x, current_p.y, current_p.z), Eigen::Vector3f(current_dp.x, current_dp.y, current_dp.z), dt);
        // w_estimation_rot_eig = ude.EstimateDisturbance_rot(Eigen::Quaternionf(current_q.q0, current_q.q1, current_q.q2, current_q.q3), Eigen::Vector3f(current_omega), dt);
    } else if(observerMode == MyLaw::ObserverMode_t::Luenberger){
        luenberger.u_thrust = Eigen::Vector3f(u_thrust.x, u_thrust.y, u_thrust.z);
        luenberger.u_torque = Eigen::Vector3f(u_torque.x, u_torque.y, u_torque.z);
        w_estimation_trans_eig = luenberger.EstimateDisturbance_trans(Eigen::Vector3f(current_p.x, current_p.y, current_p.z), Eigen::Vector3f(current_dp.x, current_dp.y, current_dp.z), dt);
        // w_estimation_rot_eig = luenberger.EstimateDisturbance_rot(Eigen::Quaternionf(current_q.q0, current_q.q1, current_q.q2, current_q.q3), Eigen::Vector3f(current_omega), dt);
    } else if(observerMode == MyLaw::ObserverMode_t::SuperTwist){
        superTwist.u_thrust = Eigen::Vector3f(u_thrust.x, u_thrust.y, u_thrust.z);
        superTwist.u_torque = Eigen::Vector3f(u_torque.x, u_torque.y, u_torque.z);
        w_estimation_trans_eig = superTwist.EstimateDisturbance_trans(Eigen::Vector3f(current_p.x, current_p.y, current_p.z), Eigen::Vector3f(current_dp.x, current_dp.y, current_dp.z), dt);
        // w_estimation_rot_eig = superTwist.EstimateDisturbance_rot(Eigen::Quaternionf(current_q.q0, current_q.q1, current_q.q2, current_q.q3), Eigen::Vector3f(current_omega), dt);
    }else if(observerMode == MyLaw::ObserverMode_t::SlidingMode){
        slidingMode.u_thrust = Eigen::Vector3f(u_thrust.x, u_thrust.y, u_thrust.z);
        slidingMode.u_torque = Eigen::Vector3f(u_torque.x, u_torque.y, u_torque.z);
        w_estimation_trans_eig = slidingMode.EstimateDisturbance_trans(Eigen::Vector3f(current_p.x, current_p.y, current_p.z), Eigen::Vector3f(current_dp.x, current_dp.y, current_dp.z), dt);
        // w_estimation_rot_eig = slidingMode.EstimateDisturbance_rot(Eigen::Quaternionf(current_q.q0, current_q.q1, current_q.q2, current_q.q3), Eigen::Vector3f(current_omega), dt);
    }

    w_estimation_trans = Vector3Df(w_estimation_trans_eig.x(),w_estimation_trans_eig.y(),w_estimation_trans_eig.z());
    w_estimation_rot = Vector3Df(w_estimation_rot_eig.x(),w_estimation_rot_eig.y(),w_estimation_rot_eig.z());
    if(!isDisturbanceActive){
        w_estimation_trans = Vector3Df(0,0,0); 
    }

    if(!isDisturbanceRotActive){
        w_estimation_rot = Vector3Df(0,0,0);
    }

    UpdateTranslationControl(current_p, current_dp, current_q);
    UpdateThrustControl(current_p, current_dp);

    Vector3Df nada (0,0,0);
    Quaternion nadaQ (0,0,0);

    #ifdef SAVE_REAL_STATE_SPACE_CSV
        SaveStateCSV(current_p, current_dp, nada, nada, current_omega, nadaQ, current_q, dt); 
    #endif

    firstUpdate = false;
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

void MyLaw::SaveStateCSV(Vector3Df &p, Vector3Df &dp,Vector3Df &ddp, Vector3Df &domega, Vector3Df &omega, Quaternion &dq, Quaternion &q, float &dt){
    if (translationOutputFileCSV.is_open()) {
        translationOutputFileCSV    << std::fixed << std::setprecision(6)
                                    << p.x << "," << p.y << "," << p.z << ","
                                    << dp.x << "," << dp.y << "," << dp.z << ","
                                    << ddp.x << "," << ddp.y << "," << ddp.z << "," << dt << "\n";
    } else {
        std::cerr << "Error opening RealStateSpace_trans.csv\n";
    }
    if (rotationOutputFileCSV.is_open()) {
        rotationOutputFileCSV   << domega.x << "," << domega.y << "," << domega.z << ","
                                << omega.x << "," << omega.y << "," << omega.z << ","
                                << dq.q0 << "," << dq.q1 << "," << dq.q2 << "," << dq.q3 << ","
                                << q.q0 << "," << q.q1 << "," << q.q2 << "," << q.q3 << "," << dt << "\n";
    } else {
        std::cerr << "Error opening RealStateSpace_rot.csv\n";
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


