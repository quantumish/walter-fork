#pragma once

#include <cstdint>
#include <array>
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

enum Level {
    LOWLEVEL = 0xFF, //<! Control over individual motors 
    HIGHLEVEL = 0x00, //<! Higher level operations like walking and "forward" 
};

struct RobotInfo {
    uint16_t id;
    uint32_t sn;  //<! Isn't documented.
    uint8_t bandwidth; 
    uint8_t mode; //<! Isn't documented.
};

struct State {
    Level level;
    struct RobotInfo {
	uint16_t id;
	uint32_t sn;  //<! Isn't documented.
	uint8_t bandwidth; 
	uint8_t mode; //<! Isn't documented.
    } bot_info;
    struct PosInfo {
	float forward_speed;
	float side_speed;
	float rotate_speed;
	float updown_speed;
	float body_height;
	float forward_pos; //!< Front/rear displacement integrated from speed info on the robot.
	float side_pos; //!< Left/right displacement integrated from speed info on the robot.
    } pos_info;
};
