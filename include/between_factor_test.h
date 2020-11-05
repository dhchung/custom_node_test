#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

using namespace gtsam;

class BetweenFactorTest: public NoiseModelFactor2<Pose3, Pose3> {

private:
    Pose3 measured_;

public:
    BetweenFactorTest();
    BetweenFactorTest(Key key1, Key key2, const Pose3& measured,
                      const SharedNoiseModel& model = nullptr);

    Vector evaluateError(const Pose3& p1, const Pose3& p2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;
};