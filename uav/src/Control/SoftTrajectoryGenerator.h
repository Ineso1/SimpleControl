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
    Eigen::VectorXf coeffs_x;
    Eigen::VectorXf coeffs_y;
    Eigen::VectorXf coeffs_z;
    float startTime;
    float endTime;

    TrajectorySegment(const Eigen::VectorXf& cx, const Eigen::VectorXf& cy, const Eigen::VectorXf& cz, float start, float end)
        : coeffs_x(cx), coeffs_y(cy), coeffs_z(cz), startTime(start), endTime(end) {}
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

#endif // SOFT_TRAJECTORY_GENERATOR_H
