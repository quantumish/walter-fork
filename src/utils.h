#pragma once

#include <Eigen/Dense>

#define CONCAT(a, b) CONCAT_INNER(a, b)
#define CONCAT_INNER(a, b) a ## b
#define Expects(x) boost::contract::check CONCAT(contract, __COUNTER__) = boost::contract::function().precondition([&] { BOOST_CONTRACT_ASSERT(x); })
#define Ensures(x) boost::contract::check CONCAT(contract, __COUNTER__) = boost::contract::function().postcondition([&] { BOOST_CONTRACT_ASSERT(x); })
#define sgn(x) (x > 0) - (x < 0)

struct Pose {
    Eigen::Vector2f position;
    float angle;
};
