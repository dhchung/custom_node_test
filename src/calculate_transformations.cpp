#include "calculate_transformations.h"

CalTransform::CalTransform(){
}

CalTransform::~CalTransform(){

}

void CalTransform::xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4f * T){

    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;

}

void CalTransform::xyzrpy2t(std::vector<float> state, Eigen::Matrix4f * T){

    float x = state[0];
    float y = state[1];
    float z = state[2];
    float roll = state[3];
    float pitch = state[4];
    float yaw = state[5];
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;

}

void CalTransform::xyzrpy2t(Eigen::VectorXf state, Eigen::Matrix4f * T){

    float x = state(0);
    float y = state(1);
    float z = state(2);
    float roll = state(3);
    float pitch = state(4);
    float yaw = state(5);
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;
}




Eigen::Matrix4f CalTransform::xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw){

    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    Eigen::Matrix4f T;

    T.setZero(4,4);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = trans;
    T(3,3) = 1.0f;

    return T;
}

Eigen::Matrix4f CalTransform::xyzrpy2t(std::vector<float> state){

    float x = state[0];
    float y = state[1];
    float z = state[2];
    float roll = state[3];
    float pitch = state[4];
    float yaw = state[5];
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    Eigen::Matrix4f T;

    T.setZero(4,4);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = trans;
    T(3,3) = 1.0f;
    return T;
}

Eigen::Matrix4f CalTransform::xyzrpy2t(Eigen::VectorXf state){

    float x = state(0);
    float y = state(1);
    float z = state(2);
    float roll = state(3);
    float pitch = state(4);
    float yaw = state(5);
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    Eigen::Matrix4f T;

    T.setZero(4,4);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = trans;
    T(3,3) = 1.0f;
    return T;
}




void CalTransform::t2xyzrpy(Eigen::Matrix4f T, std::vector<float> * xyzrpy){

    Eigen::Matrix3f R = T.block(0,0,3,3);
    std::vector<float> rpy;
    r2rpy(R, &rpy);
    Eigen::Vector3f xyz = T.block(0,3,3,1);

    xyzrpy->clear();
    xyzrpy->push_back(xyz(0));
    xyzrpy->push_back(xyz(1));
    xyzrpy->push_back(xyz(2));
    xyzrpy->insert(xyzrpy->end(), rpy.begin(), rpy.end());

}

void CalTransform::t2xyzrpy(Eigen::Matrix4f T, Eigen::VectorXf * xyzrpy){

    Eigen::Matrix3f R = T.block(0,0,3,3);
    Eigen::Vector3f rpy;
    r2rpy(R, &rpy);
    Eigen::Vector3f xyz = T.block(0,3,3,1);


    xyzrpy->resize(6);
    xyzrpy->Zero(6);

    xyzrpy->segment(0,3) = xyz;
    xyzrpy->segment(3,3) = rpy;
}


Eigen::VectorXf CalTransform::t2xyzrpy(Eigen::Matrix4f T){

    Eigen::Matrix3f R = T.block(0,0,3,3);
    Eigen::Vector3f rpy;

    r2rpy(R, &rpy);
    Eigen::Vector3f xyz = T.block(0,3,3,1);

    Eigen::VectorXf xyzrpy(6);

    xyzrpy.segment(0,3) = xyz;
    xyzrpy.segment(3,3) = rpy;

    return xyzrpy;
}


void CalTransform::rpy2r(float roll, float pitch, float yaw, Eigen::Matrix3f * R){

    Eigen::Matrix3f R_roll;
    R_roll << 1.0f, 0.0f, 0.0f,
              0.0f, cos(roll), -sin(roll),
              0.0f, sin(roll), cos(roll);

    Eigen::Matrix3f R_pitch;
    R_pitch << cos(pitch), 0.0f, sin(pitch),
               0.0f, 1.0f, 0.0f,
               -sin(pitch), 0.0f, cos(pitch);
   
    Eigen::Matrix3f R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0.0f,
             sin(yaw), cos(yaw), 0.0f,
             0.0f, 0.0f, 1.0f;

    R->operator=(R_yaw*R_pitch*R_roll);

}

void CalTransform::r2rpy(Eigen::Matrix3f R, std::vector<float> * rpy){

    float roll;
    float pitch;
    float yaw;


    if(R(3,1) != 1.0f || R(3,1) != -1.0f) {
        pitch = -asin(R(2,0));
        roll = atan2(R(2,1)/cos(pitch), R(2,2)/cos(pitch));
        yaw = atan2(R(1,0)/cos(pitch), R(0,0)/cos(pitch));
    } else {
        yaw = 0.0f;
        if(R(2,0)==-1){
            pitch = M_PI/2.0f;
            roll = yaw + atan2(R(0,1), R(0,2));
        } else {
            pitch = -M_PI/2.0f;
            roll = -yaw + atan2(-R(0,1), -R(0,2));
        }
    }

    rpy->push_back(roll);
    rpy->push_back(pitch);
    rpy->push_back(yaw);
}

void CalTransform::r2rpy(Eigen::Matrix3f R, Eigen::Vector3f * rpy){

    float roll;
    float pitch;
    float yaw;


    if(R(2,0) != 1.0f || R(2,0) != -1.0f) {
        pitch = -asin(R(2,0));
        roll = atan2(R(2,1)/cos(pitch), R(2,2)/cos(pitch));
        yaw = atan2(R(1,0)/cos(pitch), R(0,0)/cos(pitch));
    } else {
        yaw = 0.0f;
        if(R(2,0)==-1){
            pitch = M_PI/2.0f;
            roll = yaw + atan2(R(0,1), R(0,2));
        } else {
            pitch = -M_PI/2.0f;
            roll = -yaw + atan2(-R(0,1), -R(0,2));
        }
    }

    *rpy = Eigen::Vector3f{roll, pitch, yaw};
}



void CalTransform::inverse_t(Eigen::Matrix4f T1, Eigen::Matrix4f * T2){
    T2->setZero(4,4);
    Eigen::Matrix3f R = T1.block(0,0,3,3);
    Eigen::Vector3f trans = T1.block(0,3,3,1);

    T2->block(0,0,3,3) = R.transpose();
    T2->block(0,3,3,1) = -R.transpose()*trans;
    T2->operator()(3,3) = 1.0f;
}

Eigen::Matrix4f CalTransform::inverse_t(Eigen::Matrix4f T){
    Eigen::Matrix3f R = T.block(0,0,3,3);
    Eigen::Vector3f trans = T.block(0,3,3,1);

    Eigen::Matrix4f result = Eigen::Matrix4f::Zero(4,4);

    result.block(0,0,3,3) = R.transpose();
    result.block(0,3,3,1) = -R.transpose()*trans;
    result(3,3) = 1.0f;
    return result;
}

Eigen::VectorXf CalTransform::state_transition(Eigen::VectorXf & prev_state,
                                               Eigen::VectorXf & d_state){
    Eigen::VectorXf cur_state(6);
    cur_state.Zero(6);
    Eigen::Matrix4f prev_state_T = xyzrpy2t(prev_state);
    Eigen::Matrix4f d_state_T = xyzrpy2t(d_state);

    cur_state = t2xyzrpy(d_state_T*prev_state_T);

    return cur_state;
}