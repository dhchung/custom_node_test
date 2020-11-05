#pragma once
#include <gtsam/config.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>


using namespace gtsam;

template <typename A1, typename A2,
          typename B = typename A<A1, A2>::result_type,
          typename R = typename C<A1, A2>::result_type>

class CustomPose3{
public:
    Rot3 R;
    Point3 t;

    CustomPose3 operator*(const CustomPose3 &T) const;

    
}