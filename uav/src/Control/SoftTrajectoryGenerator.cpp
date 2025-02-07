#include "SoftTrajectoryGenerator.h"

SoftTrajectoryGenerator::SoftTrajectoryGenerator() : currentTime(0.0f) {}
SoftTrajectoryGenerator::~SoftTrajectoryGenerator() {}

void SoftTrajectoryGenerator::addWaypoint(const Eigen::Vector3f& position, float time) {
    waypoints.push_back({position, time});
}

void SoftTrajectoryGenerator::generateTrajectories() {
    trajectorySegments.clear();
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        auto& wp0 = waypoints[i];
        auto& wp1 = waypoints[i + 1];
        
        Eigen::VectorXf coeffs_x = computeQuinticCoefficients(wp0.time, wp1.time, wp0.position.x(), wp1.position.x(), 0, 0, 0, 0);
        Eigen::VectorXf coeffs_y = computeQuinticCoefficients(wp0.time, wp1.time, wp0.position.y(), wp1.position.y(), 0, 0, 0, 0);
        Eigen::VectorXf coeffs_z = computeQuinticCoefficients(wp0.time, wp1.time, wp0.position.z(), wp1.position.z(), 0, 0, 0, 0);

        trajectorySegments.push_back({coeffs_x, coeffs_y, coeffs_z, wp0.time, wp1.time});
    }
}

void SoftTrajectoryGenerator::getNextState(float dt, Eigen::Vector3f& position, Eigen::Vector3f& velocity) {
    currentTime += dt;
    for (const auto& segment : trajectorySegments) {
        if (segment.startTime <= currentTime && currentTime <= segment.endTime) {
            position.x() = evaluatePolynomial(segment.coeffs_x, currentTime);
            position.y() = evaluatePolynomial(segment.coeffs_y, currentTime);
            position.z() = evaluatePolynomial(segment.coeffs_z, currentTime);
            
            velocity.x() = evaluatePolynomial(segment.v_coeffs_x, currentTime);
            velocity.y() = evaluatePolynomial(segment.v_coeffs_y, currentTime);
            velocity.z() = evaluatePolynomial(segment.v_coeffs_z, currentTime);
            return;
        }
    }
    position = waypoints.back().position;
    velocity.setZero();
}

float SoftTrajectoryGenerator::evaluatePolynomial(const Eigen::VectorXf& coeffs, float t) {
    float result = 0.0f;
    float power = 1.0f;
    for (int i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * power;
        power *= t;
    }
    return result;
}

Eigen::VectorXf SoftTrajectoryGenerator::computeQuinticCoefficients(float t0, float tf, float p0, float pf, float v0, float vf, float a0, float af) {
    Eigen::Matrix<float, 6, 6> A;
    Eigen::Matrix<float, 6, 1> b;
    
    A << 1, t0, t0*t0, t0*t0*t0, t0*t0*t0*t0, t0*t0*t0*t0*t0,
         1, tf, tf*tf, tf*tf*tf, tf*tf*tf*tf, tf*tf*tf*tf*tf,
         0, 1, 2*t0, 3*t0*t0, 4*t0*t0*t0, 5*t0*t0*t0*t0,
         0, 1, 2*tf, 3*tf*tf, 4*tf*tf*tf, 5*tf*tf*tf*tf,
         0, 0, 2, 6*t0, 12*t0*t0, 20*t0*t0*t0,
         0, 0, 2, 6*tf, 12*tf*tf, 20*tf*tf*tf;
    
    b << p0, pf, v0, vf, a0, af;
    return A.colPivHouseholderQr().solve(b);
}