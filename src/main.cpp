#include <iostream>
#include <Eigen/Dense>
#include <iterator>
#include <random>
#include "math.h"
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Regular headers
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include "calculate_transformations.h"
#include <gtsam/slam/BearingRangeFactor.h>


#include "between_factor_test.h"

using namespace std;
using namespace gtsam;

int main()
{

    NonlinearFactorGraph graph;
    CalTransform c_trans;


    const float mean = 0.0f;
    const float stddev = 2.0f * M_PI / 180.0f;

    default_random_engine generator;
    normal_distribution<float> dist(mean, stddev);

    vector<Eigen::VectorXf> states;

    Eigen::VectorXf state = Eigen::VectorXf::Zero(6);
    // state.Zero(6);

    std::cout<<state<<std::endl;

    states.push_back(state);

    // gtsam::Rot3 state_rot = gtsam::Rot3::rpy(state(3), state(4), state(5));
    gtsam::Rot3 state_rot = gtsam::Rot3::ypr(state(5), state(4), state(3));
    gtsam::Point3 state_tran(state(0), state(1), state(2));
    gtsam::Pose3 state_pose(state_rot, state_tran);


    int idx = 0;

    float p_noise_position = 0.1;
    float p_noise_ang = 1.0 * float(M_PI) / 180.0f;
    noiseModel::Diagonal::shared_ptr priorNoise =
        noiseModel::Diagonal::Sigmas((Vector(6)<<p_noise_position,
                                                 p_noise_position,
                                                 p_noise_position,
                                                 p_noise_ang,
                                                 p_noise_ang,
                                                 p_noise_ang).finished());

    graph.add(PriorFactor<Pose3>(idx, state_pose, priorNoise));

    float dx = 0.0f;
    float dy = 0.0f;
    float dz = 0.0f;

    float droll = 0.0f;
    float dpitch = 0.0f;
    float dyaw = 0.0f;

    Values initials;

    initials.insert(idx, state_pose);

    for (int ts = 1; ts < 100; ++ts)
    {
        //Only y and yaw changes
        dy = 0.1f; //[m]
        dyaw = dist(generator);

        Eigen::VectorXf d_state(6);
        d_state << dx, dy, dz, droll, dpitch, dyaw;


        std::cout<<state<<std::endl;
        std::cout<<d_state<<std::endl;

        state = c_trans.state_transition(state, d_state);
        states.push_back(state);

        Eigen::Vector4f measure;
        Eigen::Matrix4f state_T = c_trans.xyzrpy2t(state);

        Rot3 dstate_rot = gtsam::Rot3::ypr(droll, dpitch, dyaw);
        std::cout<<dstate_rot<<std::endl;
        Point3 dstate_tran = Point3(dx, dy, dz);
        // graph.add(BetweenFactor<Pose3>(idx, ++idx,Pose3(dstate_rot, dstate_tran), priorNoise));
        graph.add(boost::make_shared<BetweenFactorTest>(idx, ++idx,Pose3(dstate_rot, dstate_tran), priorNoise));

        std::cout<<state<<std::endl;


        state_rot = gtsam::Rot3::ypr(state(5), state(4), state(3));
        state_tran = Point3(state(0), state(1), state(2));
        state_pose = Pose3(state_rot, state_tran);

        initials.insert(idx, state_pose);

    }

    Values results = LevenbergMarquardtOptimizer(graph, initials).optimize();
    results.print("Final Result:\n");
    std::cout<<state<<std::endl;

}
