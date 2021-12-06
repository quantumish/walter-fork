#pragma once
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <Eigen/Dense>
#include <functional>
#include <cmath>
#include "utils.h"

namespace a1 = UNITREE_LEGGED_SDK;
constexpr double pi = M_PI;

struct InstructionOutput {    
    a1::HighCmd cmd; 
    bool done;
};

/** 
 * Lambda that takes in initial state and current state (in that order)
 * and outputs a command for the current timestamp and whether the action is done. 
 * @see Bot::RobotControl()
 */
using Instruction = std::function<InstructionOutput(Pose, a1::HighState)>;

class Bot
{
    a1::Safety safe; //!< Specifies operation mode?
    a1::UDP udp; //!< UDP struct that stores payloads to send/recv.
    a1::HighCmd cmd = {0}; //!< Command to issue to robot's UDP server.
    a1::HighState state = {0}; //!< State recieved from robot's UDP server.
    Pose initial_pose; //!< Initial pose of current instruction. @see Bot::RobotControl
    int motiontime = 0; //!< Amount of time that has passed.
    float dt = 0.002; //!< Timestep for threads. Allowed range: 0.001~0.01.   
    int index = 0; //!< Index of current instruction to execute.   
    bool executing = false; //!< Whether or not an instruction is in progress.

    /** 
     * List of Instructions to be executed sequentially by the A1.
     * @see Bot::RobotControl()
     * @see Instruction
     */
    std::vector<Instruction> instructions;
    
    /** 
     * Updates internal UDP struct.
     * Ran repeatedly in a thread at runtime. @see Bot::execute()
     */
    void UDPRecv();

    /**
     * Sends contents of internal UDP struct.
     * Ran repeatedly in a thread at runtime. @see Bot::execute()
     */
    void UDPSend();

    /** 
     * Executes each instruction. 
     * Ran repeatedly in a thread at runtime. @see Bot::execute()
     */
    void RobotControl();

    /**
     * Guesses current position.
     */
    Pose estimate_pose(Pose prev, a1::HighState state);
    /** 
     * Ensures a HighCmd is safe to run. 
     */
    bool validate_cmd(a1::HighCmd cmd);    

public:
    Bot(): safe(a1::LeggedType::A1), udp(a1::HIGHLEVEL){
        udp.InitCmdData(cmd);
    }

    /**
     * Begins issuing instructions to the robot.
     * Initializes the UDP threads and the control loop, then forces main thread to wait.
     * Only call when all instructions have been specified.
     */
    void execute();

    /**
     * @returns state of the robot represented as a State.
     */
    State get_state();
    
    /** 
     * Adds an instruction to move in the x direction.
     * @param distance Relative distance (in meters) to walk along x axis.
     * @param velocity Velocity of robot while walking. -0.7 to 1 m/s.
     */
    void move_x(float distance, float velocity);

    /** 
     * Adds an instruction to move in the y direction.
     * @param distance Relative distance (in meters) to strafe along y axis.
     * @param velocity Velocity of robot while walking. -0.4 to 0.4 m/s.
     */
    void move_y(float distance, float velocity);

    /** 
     * Adds an instruction to move.
     * @param position Position to walk to in meters relative to robot.
     * @param velocity Velocity of robot while walking. -0.4 to 0.4 m/s.
     */    
    void move(Eigen::Vector2f position, Eigen::Vector2f velocity);

    /** 
     * Adds an instruction to move. Rotates first, does no strafing.
     * @param position Position to walk to in meters relative to robot.
     * @param velocity Velocity of robot while walking. -0.4 to 0.4 m/s.
     * @param omega Magnitude of angular velocity (in rad/s). 0 to 2pi/3 rad/s.
     * @see Bot::move()
     */
    void smooth_move(Eigen::Vector2f position, float velocity, float omega = 1*pi/3);
    
    void spline_move(); 
    
    /**
     * Adds an instruction to rotate in place.
     * @param theta Angle to rotate (in rad).
     * @param omega Angular velocity (in rad/s). -2pi/3 to 2pi/3 rad/s.
     */
    void rotate(float theta, float omega);

    /** 
     * Messes with the lights on the robot, which the manual explicitly says not to do.
     * You probably shouldn't run this.
     * @param lights RGB values for each light.
     */ 
    [[deprecated("unsafe")]] void set_led(std::vector<Eigen::Vector3i> lights);

    /**
     * Sets pitch/yaw/roll of bot's body. Pretty useless for now.
     */
    [[deprecated]] void set_pyr(Eigen::Vector3d pitch, Eigen::Vector3d yaw, Eigen::Vector3d roll);
    /** 
     * Sets height of bot's body from the ground. Pretty useless for now.
     */
    [[deprecated]] void set_height(float h, float v);
};
