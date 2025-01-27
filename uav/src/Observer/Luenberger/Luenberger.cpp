#include "Luenberger.h"

namespace Observer {

Luenberger::Luenberger() {
    firstIteration_trans = true;
    firstIteration_rot = true;

    // Initialize extended matrices for translational dynamics
    A_ext_trans = Eigen::MatrixXf::Zero(12, 12);
    B_ext_trans = Eigen::MatrixXf::Zero(12, 3);
    C_ext_trans = Eigen::MatrixXf::Zero(3, 12);

    // Initialize extended matrices for rotational dynamics
    A_ext_rot = Eigen::MatrixXf::Zero(12, 12);
    B_ext_rot = Eigen::MatrixXf::Zero(12, 3);
    C_ext_rot = Eigen::MatrixXf::Zero(3, 12);

    // Initialize observer gains
    L_trans = Eigen::MatrixXf::Zero(12, 3);
    L_rot = Eigen::MatrixXf::Zero(12, 3);

    // Initialize state estimates
    x_trans = Eigen::VectorXf::Zero(12);
    x_rot = Eigen::VectorXf::Zero(12);

    InitializeEstimates();
}

Luenberger::~Luenberger() {}

void Luenberger::InitializeEstimates() {
    x_trans.setZero();
    x_rot.setZero();

    A_ext_trans.block<6, 6>(0, 0) = A_trans;
    A_ext_trans.block<6, 3>(0, 6) = B_trans;
    A_ext_trans.block<3, 3>(6, 9) = Eigen::MatrixXf::Identity(3, 3);
    A_ext_trans.block<3, 3>(9, 6) =  0* -Eigen::MatrixXf::Identity(3, 3);

    A_ext_rot.block<6, 6>(0, 0) = A_rot;
    A_ext_rot.block<6, 3>(0, 6) = B_rot;
    A_ext_rot.block<3, 3>(6, 9) = Eigen::MatrixXf::Identity(3, 3);
    A_ext_rot.block<3, 3>(9, 6) = 0* -Eigen::MatrixXf::Identity(3, 3);

    B_ext_trans.block<6, 3>(0, 0) = B_trans;
    B_ext_rot.block<6, 3>(0, 0) = B_rot;

    C_ext_trans.block<3, 6>(0, 0) = C_trans; 

    C_ext_rot.block<3, 6>(0, 0) = C_rot; 

    L_trans <<
        25.9999999999889,   -1.20656765351465e-11,   -9.19927558605081e-11,
        -1.74277389339572e-11,   26.0000000000021,   -1.73250099252020e-11,
         3.99169888498837e-11,    2.37823881820265e-12,    26.0000000000056,
       250.999999999791,   -2.03717970036072e-10,   -1.75860609406701e-09,
       -3.29527772728847e-10,   251.000000000046,   -3.31341387659679e-10,
         7.94813944244144e-10,    4.83756001452969e-11,   251.000000000107,
       431.729999999475,   -4.50115405931106e-10,   -4.45121594339819e-09,
       -8.26182793669817e-10,   431.730000000132,   -8.37856050129089e-10,
         2.08623029406316e-09,    1.28787066799578e-10,   431.730000000273,
       680.399999998924,   -7.96819450690498e-10,   -9.09412626188640e-09,
       -1.67267314583956e-09,   680.400000000302,   -1.70691868937195e-09,
         4.40646889634985e-09,    2.73968158380886e-10,   680.400000000558;



    L_rot << 
        29.2447394507054,   -1.10830200213826,    5.31961914905072,
       -1.78032833932902,   27.6958936893007,    0.657191229945013,
        1.58378767428938,    0.391550792739331,   28.0593668599922,
      307.878823180379,  -24.4393311896854,  115.617709896587,
      -40.4162772925490,  274.117589180155,    9.86649743529291,
       33.9351357454262,    7.89347075498088,  280.981890775826,
        2.84872636856943,   -0.290066065153214,    1.65875653497531,
       -0.535023037098857,    2.37432390745997,    0.100971276790076,
        0.921888803947273,    0.196391935195723,    4.75857527198086,
        4.41714204187767,   -0.715420713273649,    3.49068885422631,
       -1.33138157586819,    3.40529265819842,   -0.0825426438771583,
        1.88068708144691,    0.340228730102512,    6.76023048513600;
}

void Luenberger::SetNewGainL_trans(const Eigen::MatrixXf& L_trans_new) {
    L_trans = L_trans_new;
}

void Luenberger::SetNewGainL_rot(const Eigen::MatrixXf& L_rot_new) {
    L_rot = L_rot_new;
}

Eigen::Vector3f Luenberger::EstimateDisturbance_trans(const Eigen::Vector3f& p, const Eigen::Vector3f& dp, float dt) {
    Eigen::VectorXf x_t(12);
    x_t << p, dp, Eigen::VectorXf::Zero(6);
    Eigen::Vector3f y = C_ext_trans * x_t;
    Eigen::VectorXf dx_hat_ext_L_trans = 
        A_ext_trans * x_trans + 
        B_ext_trans * (u_thrust - Eigen::Vector3f(0, 0, g * mass)) + 
        L_trans * (y - C_ext_trans * x_trans);

    x_trans += dt * dx_hat_ext_L_trans;
    Eigen::Vector3f disturbance_estimate = x_trans.segment<3>(6);
    SaveStateEstimationCSV(x_trans, dx_hat_ext_L_trans, disturbance_estimate, "TranslationalEstimation.csv");
    return disturbance_estimate;
    return Eigen::Vector3f(0,0,0);
}


Eigen::Vector3f Luenberger::EstimateDisturbance_rot(const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) {
    Eigen::Vector3f u_torque_L = u_torque - omega.cross(J * omega);
    Eigen::VectorXf x_r(12);
    Eigen::Vector3f q_rvec = rotvec(q);
    x_r << q_rvec, omega, Eigen::VectorXf::Zero(6);
    Eigen::Vector3f y = C_ext_rot * x_r;
    Eigen::VectorXf dx_hat = A_ext_rot * x_rot + B_ext_rot * u_torque_L + L_rot * (y - C_ext_rot * x_rot);
    x_rot += dt * dx_hat;
    Eigen::Vector3f disturbance_estimate = x_rot.segment<3>(6);
    SaveStateEstimationCSV(x_rot, dx_hat, disturbance_estimate, "RotationalEstimation.csv");
    return disturbance_estimate;
}

} // namespace Observer
