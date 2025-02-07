#ifndef SOFT_TRAJECTORY_GENERATOR_H
#define SOFT_TRAJECTORY_GENERATOR_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

class SoftTrajectoryGenerator {
public:
    struct Waypoint {
        Eigen::Vector3f position;
        float time;
    };

    struct TrajectorySegment {
        Eigen::VectorXf coeffs_x, coeffs_y, coeffs_z;
        Eigen::VectorXf v_coeffs_x, v_coeffs_y, v_coeffs_z;
        float startTime, endTime;

        TrajectorySegment(const Eigen::VectorXf& cx, const Eigen::VectorXf& cy, const Eigen::VectorXf& cz, float start, float end)
            : coeffs_x(cx), coeffs_y(cy), coeffs_z(cz), startTime(start), endTime(end) {
            v_coeffs_x = computeDerivative(cx);
            v_coeffs_y = computeDerivative(cy);
            v_coeffs_z = computeDerivative(cz);
        }

        static Eigen::VectorXf computeDerivative(const Eigen::VectorXf& coeffs) {
            Eigen::VectorXf deriv_coeffs(coeffs.size() - 1);
            for (int i = 1; i < coeffs.size(); ++i) {
                deriv_coeffs[i - 1] = i * coeffs[i];
            }
            return deriv_coeffs;
        }
    };

    SoftTrajectoryGenerator();
    ~SoftTrajectoryGenerator();

    void addWaypoint(const Eigen::Vector3f& position, float time);
    void generateTrajectories();
    void getNextState(float dt, Eigen::Vector3f& position, Eigen::Vector3f& velocity);

private:
    std::vector<Waypoint> waypoints;
    std::vector<TrajectorySegment> trajectorySegments;
    float currentTime;
    float evaluatePolynomial(const Eigen::VectorXf& coeffs, float t);
    Eigen::VectorXf computeQuinticCoefficients(float t0, float tf, float p0, float pf, float v0, float vf, float a0, float af);
};

#endif