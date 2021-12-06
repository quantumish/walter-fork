#include <cmath>
#include <Eigen/Dense>
#include <boost/contract.hpp>
#include <unistd.h>
#include "bot.h"
#include "utils.h"

void Bot::UDPRecv() {
    udp.Recv();
}

void Bot::UDPSend() {
    udp.Send();
}

State Bot::get_state() {
    return {
	.level = Level::HIGHLEVEL,
	.bot_info = {
	    .id = state.robotID, 
	    .sn = state.SN,
	    .bandwidth = state.bandWidth,
	    .mode = state.mode,
	},
	.pos_info = {
	    .forward_speed = state.forwardSpeed,
	    .side_speed = state.sideSpeed,
	    .rotate_speed = state.rotateSpeed,
	    .updown_speed = state.updownSpeed,
	    .body_height = state.bodyHeight,
	    .forward_pos = state.forwardPosition,
	    .side_pos = state.sidePosition,	    
	},
    };
}

Eigen::Vector3f pyr_from_quaternion(float* quaternion) {
    float qw = quaternion[0];
    float qx = quaternion[1];
    float qy = quaternion[2];
    float qz = quaternion[3];
    Eigen::Vector3f out;
    out << static_cast<float>(atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)),
	static_cast<float>(asin(-2.0*(qx*qz - qw*qy))),
	static_cast<float>(atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz));
    return out;
}

Pose Bot::estimate_pose(Pose prev, a1::HighState state) {
    return {
	.position {
	    state.forwardSpeed*dt + 0.5*state.imu.accelerometer[0]*(dt*dt),
	    state.sideSpeed*dt + 0.5*state.imu.accelerometer[1]*(dt*dt)
	},
	.angle = pyr_from_quaternion(state.imu.quaternion)[1]-prev.angle,
    };
}

// TODO: Prevent abuse of cmd.led and cmd.crc
bool Bot::validate_cmd(a1::HighCmd cmd) {
    return
	cmd.levelFlag == 0x00 &&
	(cmd.mode == 1 || cmd.mode == 2) &&
	(cmd.forwardSpeed > -1 && cmd.forwardSpeed < 1) &&
	(cmd.sideSpeed > -1 && cmd.sideSpeed < 1) &&
	(cmd.rotateSpeed > -1 && cmd.rotateSpeed < 1) &&
	(cmd.bodyHeight > -1 && cmd.bodyHeight < 1) &&
	(cmd.yaw > -1 && cmd.yaw < 1) &&
	(cmd.pitch > -1 && cmd.pitch < 1) &&
	(cmd.roll > -1 && cmd.roll < 1);
}

void Bot::execute() {
    a1::LoopFunc loop_control("control_loop", dt, boost::bind(&Bot::RobotControl, this));
    a1::LoopFunc loop_udpSend("udp_send", dt, boost::bind(&Bot::UDPSend, this));
    a1::LoopFunc loop_udpRecv("udp_recv", dt, boost::bind(&Bot::UDPRecv, this));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    sleep(5);
    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    loop_control.shutdown();
}

void Bot::RobotControl() {
    motiontime += 2;
    udp.GetRecv(state);
    InstructionOutput out;
    if (!executing) {
	initial_pose = {{0,0}, pyr_from_quaternion(state.imu.quaternion)[1]};
	out = instructions[index](initial_pose, state);
	executing = true;
    } else {
	out = instructions[index](initial_pose, state);
    }
    if (!out.done) {
	cmd = out.cmd;
    } else {
	executing = false;
	index++;
    }
    if (validate_cmd(cmd)) udp.SetSend(cmd);
}

void Bot::move_y(float d, float v) {
    Expects(v > -0.7 && v < 1);
    instructions.push_back(
	[d, v](Pose initial, a1::HighState state) {
	    a1::HighCmd cmd {0};
	    cmd.mode = 2;
	    (v < 0) ? cmd.forwardSpeed = v/0.7 : cmd.forwardSpeed = v;
	    return InstructionOutput{cmd, true};
	}
    );
}

void Bot::move_x(float d, float v) {
    Expects(v > -0.4 && v < 0.4);
    instructions.push_back(
	[d, v](Pose initial, a1::HighState state) {
	    a1::HighCmd cmd {0};
	    cmd.mode = 2;
	    cmd.sideSpeed = v;
	    return InstructionOutput{cmd, true};
	}
    );
}

void Bot::move(Eigen::Vector2f pos, Eigen::Vector2f v) {
    Expects((v[0] > -0.4 && v[0] < 0.4) && (v[1] > -0.7 && v[1] < 1));
    instructions.push_back(
	[pos, v](Pose initial, a1::HighState state) {
	    a1::HighCmd cmd {0};
	    cmd.mode = 2;
	    cmd.sideSpeed = v[0];
	    cmd.forwardSpeed = v[1];
	    return InstructionOutput{cmd, true};
	}
    );
}

void Bot::smooth_move(Eigen::Vector2f pos, float v, float omega) {
    Expects((v > -0.7 && v < 1) && (-2*pi/3 < omega && omega < 2*pi/3));
    float theta = atan(pos[1]/pos[0]);
    rotate(theta, sgn(theta)*omega);
    move_y(pos.norm(), v);
}

void Bot::set_led(std::vector<Eigen::Vector3i> lights) {
    Expects(lights.size() == 4);
    instructions.push_back(
	[lights](Pose initial, a1::HighState state) {
	    a1::HighCmd cmd {0};
	    for (int i = 0; i < 4; i++) {
		// TODO avoid C-style cast.
		cmd.led[i] = *(a1::LED*)lights[i].data();
	    }
	    return InstructionOutput{cmd, true};
	}
    );
}

void Bot::rotate(float theta, float omega) {
    Expects(-2*pi/3 < omega && omega < 2*pi/3);
    instructions.push_back(
	[this, theta, omega](Pose initial, a1::HighState state) {
	    a1::HighCmd cmd {0};
	    cmd.mode = 1; // Maybe?
	    cmd.rotateSpeed = omega / (2*pi/3);	   	   
	    return (estimate_pose(initial, state).angle == theta) ?
		InstructionOutput{cmd, true} : InstructionOutput{cmd, false};
	}
    );
}
