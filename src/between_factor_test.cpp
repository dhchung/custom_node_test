#include "between_factor_test.h"

BetweenFactorTest::BetweenFactorTest(Key key1, Key key2, const Pose3 &measured,
                                     const SharedNoiseModel &model) : NoiseModelFactor2(model, key1, key2),
                                                                      measured_(measured) {}

Vector BetweenFactorTest::evaluateError(const Pose3 &p1, const Pose3 &p2,
                                        boost::optional<Matrix &> H1,
                                        boost::optional<Matrix &> H2) const
{
    Pose3 hx = traits<Pose3>::Between(p1, p2, H1, H2);
    Vector result = traits<Pose3>::Local(measured_, hx);

    // std::cout << "result : "<<std::endl<< result << std::endl;


    // if(H1){
    //     std::cout<<"H1 : "<<std::endl<<H1.get_ptr()->array()<<std::endl;

    // }    

    // if(H2){
    //     std::cout<<"H2 : "<<std::endl<<H2.get_ptr()->array()<<std::endl;

    // }    


    // std::cout<<"H2 : "<<std::endl<<H2.get_ptr()->array()<<std::endl;

    // Vector test(12);
    // test<<result, result;

    // std::cout<<"Test: "<<std::endl<<test<<std::endl;

    return result;
}