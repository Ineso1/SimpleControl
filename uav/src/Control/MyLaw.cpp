#include "MyLaw.h"

namespace flair
{
namespace filter
{
	
MyLaw::MyLaw(const LayoutPosition* position, string name) : ControlLaw(position->getLayout(),name,8) {
       
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
    input = new Matrix(this,23,1,floatType,name);
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
    dataexp = new Matrix(this,desc,floatType,name);
    delete desc;
    AddDataToLog(dataexp);

    perturbation_trans = Vector3Df(0,0,0);
    perturbation_rot = Vector3Df(0,0,0);

    controlLayout = new GridLayout(position, "PD control");
    
    GroupBox* control_groupbox_att = new GroupBox(controlLayout->LastRowLastCol(),"Attitude");
    kpatt=new Vector3DSpinBox(control_groupbox_att->NewRow(),"kpatt",0,100,1);
    kdatt=new Vector3DSpinBox(control_groupbox_att->NewRow(),"kdatt",0,100,1);
    satAtt=new DoubleSpinBox(control_groupbox_att->NewRow(),"satAtt:",0,1,0.1);
    
    GroupBox* control_groupbox_trans = new GroupBox(controlLayout->LastRowLastCol(),"Translation");
    kppos=new Vector3DSpinBox(control_groupbox_trans->NewRow(),"kppos",0,100,0.01);
    kdpos=new Vector3DSpinBox(control_groupbox_trans->NewRow(),"kdpos",0,100,0.01);
    satPos=new DoubleSpinBox(control_groupbox_trans->NewRow(),"satPos:",0,1,0.1);
    satPosForce=new DoubleSpinBox(control_groupbox_trans->NewRow(),"satPosForce:",0,1,0.1);
   
    paramsLayout = new GridLayout(controlLayout->NewRow(), "Params");
    GroupBox* params_groupbox = new GroupBox(paramsLayout->NewRow(),"Params");
    mass_layout = new DoubleSpinBox(params_groupbox->NewRow(),"Mass",0.1,10,0.001,3,0.405);
    motorConst = new DoubleSpinBox(params_groupbox->NewRow(),"Motor const",0,20,0.0001,10,10);

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

    p_d = Vector3Df(0,0,1);
    dp_d = Vector3Df(0,0,0);
}

MyLaw::~MyLaw(void) {
    if (errorsOutputFileCSV.is_open()) {
        errorsOutputFileCSV.close();
    }
}

void MyLaw::UpdateTranslationControl(){
    // Vector3Df pos_err = current_p - p_d;
    // std::cout<<"errZ "<< current_p.z << std::endl;
    // Vector3Df vel_err = current_dp - Vector3Df(0,0,0);
    // pos_err.Rotate(-current_q);
    // vel_err.Rotate(-current_q);
    // uX_custom->SetValues(pos_err.x, vel_err.x);
    // uX_custom->Update(GetTime());
    // uY_custom->SetValues(pos_err.y, vel_err.y);
    // uY_custom->Update(GetTime());
}

void MyLaw::UpdateThrustControl(){
    // Vector3Df pos_err = -current_p - p_d;
    // Vector3Df vel_err = -current_dp - Vector3Df(0,0,0);
    // uZ_custom->SetValues(pos_err.z, vel_err.z);
    // uZ_custom->Update(GetTime());
}


void MyLaw::CalculateControl(Vector3Df& current_p , Vector3Df &current_dp, Quaternion &current_q, Vector3Df &current_omega) {
    // mass = mass_layout->Value();
    
    // if(firstUpdate){
    //     previous_chrono_time = std::chrono::high_resolution_clock::now();
    // }
    // float dt;
    // auto current_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<float> alt_dt = current_time - previous_chrono_time;
    // dt = alt_dt.count();
    // previous_chrono_time = current_time;
    // if(firstUpdate){
    //     dt = 0;
    // }
    // std::cout<<dt<<std::endl;


    // float thrust = uZ_custom->Output();
    // u_thrust = Vector3Df(0,0,thrust);

    // if(observerMode == MyLaw::ObserverMode_t::UDE){
    //     /* Implementation of the observer */
    // } else if(observerMode == MyLaw::ObserverMode_t::Luenberger){
    //     /* Implementation of the observer */
    // } else if(observerMode == MyLaw::ObserverMode_t::SuperTwist){
    //     /* Implementation of the observer */
    // }else if(observerMode == MyLaw::ObserverMode_t::SlidingMode){
    //     /* Implementation of the observer */
    // }

    // /* Aqui hay que actualizat los valores*/
    // w_estimation_trans = Vector3Df(0,0,0);
    // w_estimation_rot = Vector3Df(0,0,0);


    // if(!isDisturbanceActive){
    //     w_estimation_trans = Vector3Df(0,0,0); 
    // }

    // if(!isDisturbanceRotActive){
    //     w_estimation_rot = Vector3Df(0,0,0);
    // }

    // UpdateTranslationControl(current_p, current_dp, current_q);
    // UpdateThrustControl(current_p, current_dp);

    // Vector3Df nada (0,0,0);
    // Quaternion nadaQ (0,0,0);

    // #ifdef SAVE_REAL_STATE_SPACE_CSV
    //     SaveStateCSV(current_p, current_dp, nada, nada, current_omega, nadaQ, current_q, dt); 
    // #endif

    // firstUpdate = false;
}



void MyLaw::UpdateFrom(const io_data *data) {
    input->GetMutex();
    Vector3Df p(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0));
    Vector3Df dp(input->ValueNoMutex(3,0),input->ValueNoMutex(4,0),input->ValueNoMutex(5,0));
    Quaternion q(input->ValueNoMutex(6,0),input->ValueNoMutex(7,0),input->ValueNoMutex(8,0),input->ValueNoMutex(9,0));
    Vector3Df w(input->ValueNoMutex(10,0),input->ValueNoMutex(11,0),input->ValueNoMutex(12,0));
    input->ReleaseMutex();
    
    Vector3Df kp_trans = kppos->Value();
    Vector3Df kd_trans = kdpos->Value();
    Vector3Df kp_rot = kpatt->Value();
    Vector3Df kd_rot = kdatt->Value();

    float sat_trans = satPos->Value();
    float sat_force = satPosForce->Value();
    float sat_rot = satAtt->Value();
    

    float delta_t;
    q.Normalize();

    delta_t=(float)(data->DataTime()-previous_time)/1000000000.;

    if(firstUpdate==true) {
        delta_t=0;
        firstUpdate=false;
        u_thrust.x=0;
        u_thrust.y=0;
        u_thrust.z=0.452;
    }

    float Fth;
    Vector3Df pos_err= p - p_d;
    Vector3Df vel_err= dp - dp_d;

    std::cout<<"err p:\t"<<pos_err.x<<"\t"<<pos_err.y<<"\t"<<pos_err.z<<std::endl;

    if (pos_err.GetNorm() > sat_trans){
		Vector3Df sat_pos_err = pos_err * (sat_trans / pos_err.GetNorm());
        std::cout<<"Saturado"<<std::endl;
    }

    u_thrust = (- kp_trans * pos_err) + (- kd_trans * vel_err); // - estimacion de perturbacion
	u_thrust.Saturate(sat_force);
    u_thrust.z = u_thrust.z - 0.452;
	Fth = -u_thrust.GetNorm();


    if (Fth>0){Fth = 0;}

    Quaternion qz(1,0,0,0);
    Quaternion q_d(1,0,0,0);
    if (u_thrust.GetNorm()!=0){
		
        float scalarPart = DotProduct(Vector3Df(0,0,-1), u_thrust) + u_thrust.GetNorm();
        Vector3Df vectorPart = CrossProduct(Vector3Df(0,0,-1), u_thrust);        
        q_d.q0 = scalarPart;
        q_d.q1 = vectorPart.x;
        q_d.q2 = vectorPart.y;
        q_d.q3 = vectorPart.z;
        q_d.Normalize();
        
        q_d = q_d * qz;
        q_d.Normalize();
    }else{
        q_d = q;
        q_d.Normalize();
        std::cout<<"Porque estoy aqui?"<<std::endl;
    }
	Quaternion qe = q_d.GetConjugate()*q;
	Vector3Df thetae = 2 * qe.GetLogarithm();

    Vector3Df Tau_u = (kp_trans * thetae) + (kd_rot * w);
    Vector3Df Tau = Tau_u;
    Tau.Saturate(sat_rot);

    output->SetValue(0,0,Tau.x);
    output->SetValue(1,0,Tau.y);
    output->SetValue(2,0,Tau.z);
    output->SetValue(3,0,Fth);
    output->SetDataTime(data->DataTime());
	
	output->SetValue(4,0,q_d.q0);
    output->SetValue(5,0,q_d.q1);
    output->SetValue(6,0,q_d.q2);
	output->SetValue(7,0,q_d.q3);

    // stateM->GetMutex();
    // stateM->SetValueNoMutex(0,0,q.q0);
    // stateM->SetValueNoMutex(1,0,q.q1);
    // stateM->SetValueNoMutex(2,0,q.q2);
    // stateM->SetValueNoMutex(3,0,q.q3);
    // stateM->SetValueNoMutex(4,0,qd.q0);
    // stateM->SetValueNoMutex(5,0,qd.q1);
    // stateM->SetValueNoMutex(6,0,qd.q2);
    // stateM->SetValueNoMutex(7,0,qd.q3);
    // stateM->SetValueNoMutex(8,0,w.x);
    // stateM->SetValueNoMutex(9,0,w.y);
    // stateM->SetValueNoMutex(10,0,w.z);
    // stateM->SetValueNoMutex(11,0,p.x);
    // stateM->SetValueNoMutex(12,0,p.y);
    // stateM->SetValueNoMutex(13,0,p.z);
    // stateM->SetValueNoMutex(14,0,p_d.x);
    // stateM->SetValueNoMutex(15,0,p_d.y);
    // stateM->SetValueNoMutex(16,0,p_d.z);
    // stateM->SetValueNoMutex(17,0,dp.x);
    // stateM->SetValueNoMutex(18,0,dp.y);
    // stateM->SetValueNoMutex(19,0,dp.z);
    // stateM->SetValueNoMutex(20,0,0);
    // stateM->SetValueNoMutex(21,0,0);
    // stateM->SetValueNoMutex(22,0,0);
    // stateM->ReleaseMutex();

    previous_time=data->DataTime();
    ProcessUpdate(output);    
}






void MyLaw::Reset(void) {
    p_d.x = 0;
    p_d.y = 0;
    p_d.z = 0;
    dp_d.x = 0;
    dp_d.y = 0;
    dp_d.z = 0;
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

void MyLaw::SetValues(Vector3Df& p, Vector3Df& dp, Quaternion& q,Vector3Df& w){
    input->SetValue(0,0,p.x);
    input->SetValue(1,0,p.y);
    input->SetValue(2,0,p.z);
    input->SetValue(3,0,dp.x);
    input->SetValue(4,0,dp.y);
    input->SetValue(5,0,dp.z);
    input->SetValue(6,0,q.q0);
    input->SetValue(7,0,q.q1);
    input->SetValue(8,0,q.q2);
    input->SetValue(9,0,q.q3);
    input->SetValue(10,0,w.x);
    input->SetValue(11,0,w.y);
    input->SetValue(12,0,w.z);
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


