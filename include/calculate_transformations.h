#pragma once
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>


class CalTransform{
public:
    CalTransform();
    ~CalTransform();

    void xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4f * T);
    void xyzrpy2t(std::vector<float> state, Eigen::Matrix4f * T);
    void xyzrpy2t(Eigen::VectorXf state, Eigen::Matrix4f *T);

    Eigen::Matrix4f xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw);
    Eigen::Matrix4f xyzrpy2t(std::vector<float> state);
    Eigen::Matrix4f xyzrpy2t(Eigen::VectorXf state);


    void t2xyzrpy(Eigen::Matrix4f T, std::vector<float> * xyzrpy);
    void t2xyzrpy(Eigen::Matrix4f T, Eigen::VectorXf * xyzrpy);

    Eigen::VectorXf t2xyzrpy(Eigen::Matrix4f T);

    void rpy2r(float roll, float pitch, float yaw, Eigen::Matrix3f * R);
    void r2rpy(Eigen::Matrix3f R, std::vector<float> * rpy);
    void r2rpy(Eigen::Matrix3f R, Eigen::Vector3f * rpy);


    void inverse_t(Eigen::Matrix4f T1, Eigen::Matrix4f *T2);
    Eigen::Matrix4f inverse_t(Eigen::Matrix4f T);

    Eigen::VectorXf state_transition(Eigen::VectorXf & prev_state,
                                     Eigen::VectorXf & d_state);

};