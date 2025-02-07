// Kalman.cpp
#include "KFC.h"
#include "iostream"

namespace Observer{

KFC::KFC() 
{
    dt = 0.01;
    Fk = Eigen::MatrixXf::Identity(6, 6) + (A_trans * dt) + 0.5 * ((A_trans * A_trans * dt * dt));

    Bk = dt * B_trans;
    Hk = Eigen::MatrixXf::Identity(6, 6);

    Qk = Eigen::MatrixXf::Identity(6, 6) * 1.5;
    Rk = Eigen::MatrixXf::Identity(6, 6) * 25;

    Pk = Eigen::MatrixXf::Identity(6, 6) * 60;
    Xk = Eigen::VectorXf::Zero(6);
    Xk.head<3>() = p;
    Xk.tail<3>() = dp;

    firstUpdate = true;

    ///////////////////////////////// Crear archivo para guardar las señales de entrada /////////////////////////////////
    std::ofstream outputFile1("/home/fercho_lopez/Documents/EKFC/data_in.csv");
    if (!outputFile1.is_open()) {
        std::cerr << "Error al crear el archivo data_in.csv\n(╥﹏╥)" << std::endl;
    }
    else{
        std::cout << "El archivo se creo correctamente data_in.csv\nദി( • ᴗ - ) ✧" << std::endl;
        outputFile1 << "x,y,z" << std::endl; // Encabezado del archivo CSV
    }

    ///////////////////////////////// Crear archivo para guardar resultados de Kalman /////////////////////////////////
    std::ofstream outputFile2("/home/fercho_lopez/Documents/EKFC/data_kalman.csv");
    if (!outputFile2.is_open()) {
        std::cerr << "Error al crear el archivo data_kalman.csv\n(╥﹏╥)" << std::endl;
    }
    else{
        std::cout << "El archivo se creo correctamente data_kalman.csv\n𓆝 𓆟 𓆞 𓆝 𓆟 𓆝 𓆟 𓆞 𓆝 𓆟 𓆝 𓆟 𓆞 𓆝 𓆟 𓆝 𓆟 𓆞 𓆝 𓆟" << std::endl;
        outputFile2 << "x,y,z" << std::endl; // Encabezado del archivo CSV
    }

    std::cout << "The filter is filtering ( -_•)▄︻テحكـ━一 (21) \n";

    std::cout << "Se ha generado el ruido gaussiano\n  ( ◡̀_◡́)ᕤ\n";

    initialize();
}

KFC::~KFC() {}

void KFC::resetKFC() {
    firstUpdate = true;
}

void KFC::KFC_estimate(const flair::core::Vector3Df& p_aux, const flair::core::Vector3Df& dp_aux, const flair::core::Vector3Df& u_thrust_aux)
{
    Eigen::Vector3f p(p_aux.x, p_aux.y, p_aux.z);
    Eigen::Vector3f dp(dp_aux.x, dp_aux.y, dp_aux.z);
    Eigen::Vector3f u_thrust(u_thrust_aux.x, u_thrust_aux.y, u_thrust_aux.z);

    Eigen::VectorXf X(6);
    X.head<3>() = p;
    X.tail<3>() = dp;

    // Guardar los datos de entrada en data_in.csv
    std::ofstream outputFile1("/home/fercho_lopez/Documents/EKFC/data_in.csv", std::ios::app); // Abrir en modo de adición
    if (outputFile1.is_open()) {
        outputFile1 << p.x() << "," << p.y() << "," << p.z() << std::endl; // Escribir los datos de p
    }

    if (p.isZero() & dp.isZero()) {
        Qk = Eigen::MatrixXf::Identity(6, 6) * 150;
        Rk = Eigen::MatrixXf::Identity(6, 6) * 1e10;
        X = Xk;
        //std::cout << "MAMA ESCUCHO BORROSO!\nMissing data: using model prediction" << std::endl;
    } else {
        Qk = Eigen::MatrixXf::Identity(6, 6) * 1.5;
        Rk = Eigen::MatrixXf::Identity(6, 6) * 25;
        //std::cout << "ia sirbo\n" << std::endl;
    }

    if (firstUpdate && dp.isZero() && u_thrust.isZero()) {
        Xk = Fk * Xk;
        firstUpdate = false;
    } else {
        Xk = Fk * Xk + Bk * (u_thrust - Eigen::Vector3f(0, 0, g * mass));
    }
    Pk = Fk * Pk * Fk.transpose() + Qk;

    Zk = Hk * X;
    Eigen::VectorXf Zest = Hk * Xk;
    Eigen::VectorXf Yk = Zk - Zest;

    Sk = Hk * Pk * Hk.transpose() + Rk;
    Kk = Pk * Hk.transpose() * Sk.inverse();

    Xk = Xk + Kk * Yk;
    Pk = (Eigen::MatrixXf::Identity(6, 6) - Kk * Hk) * Pk;

    // Guardar los resultados filtrados en data_kalman.csv
    std::ofstream outputFile2("/home/fercho_lopez/Documents/EKFC/data_kalman.csv", std::ios::app); // Abrir en modo de adición
    if (outputFile2.is_open()) {
        outputFile2 << Xk.head<3>().x() << "," << Xk.head<3>().y() << "," << Xk.head<3>().z() << std::endl; // Escribir los datos filtrados
    }

    outputFile1.close();
    outputFile2.close();
}

void KFC::getState(flair::core::Vector3Df& p_aux, flair::core::Vector3Df& dp_aux) const
{
    // p = Xk.head<3>();
    // dp = Xk.tail<3>();

    p_aux = flair::core::Vector3Df(p.x(), p.y(), p.z());
    dp_aux = flair::core::Vector3Df(dp.x(), dp.y(), dp.z());
}
} // namespace Observer