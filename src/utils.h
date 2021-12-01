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
    LOWLEVEL = 0xFF,
    HIGHLEVEL = 0x00,
};

struct State {
    Level level;
    struct RobotInfo {
	uint16_t id;
	uint32_t sn;
	uint8_t bandwidth;
	uint8_t mode;
    } robot_info;
    struct PosInfo {
	float forward_speed;
	float side_speed;
	float rotate_speed;
	float updown_speed;
	float body_height;
	float forward_pos;
	float side_pos;
    } pos_info;
    struct FootInfo {
	std::array<float, 3> foot_pos[4];
	std::array<float, 3> foot_speed[4];
	int16_t foot_force[4];
	int16_t foot_force_est[4];	
    } foot_info;    
};
