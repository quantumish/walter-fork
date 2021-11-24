#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <functional>
using namespace UNITREE_LEGGED_SDK;

struct InstructionOutput {    
    HighCmd cmd; 
    bool done;
};

/** 
 * Lambda that takes in initial state and current state (in that order)
 * and outputs a command for the current timestamp and whether the action is done. 
 * @see Bot::RobotControl()
 */
using Instruction = std::function<InstructionOutput(HighState, HighState)>;

class Bot
{
    Safety safe; //!< Specifies operation mode?
    UDP udp; //!< UDP struct that stores payloads to send/recv.
    HighCmd cmd = {0}; //!< Command to issue to robot's UDP server.
    HighState state = {0}; //!< State recieved from robot's UDP server.
    HighState initial_state = {0}; //!< Initial state of current instruction. @see Bot::RobotControl
    HighState true_initial_state = {0}; //!< Actual initial state of the robot used for defining a coordinate system.
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
     * Ensures a HighCmd is safe to run. 
     */
    bool validate_cmd(HighCmd cmd);    

public:
    Bot(): safe(LeggedType::A1), udp(HIGHLEVEL){
        udp.InitCmdData(cmd);
    }

    /**
     * Begins issuing instructions to the robot.
     * Initializes the UDP threads and the control loop, then forces main thread to wait.
     * Only call when all instructions have been specified.
     */
    void execute();
    
    /** 
     * Adds an instruction to walk forward.
     * @param distance Distance (in meters) to walk forward.
     * @param velocity Velocity of robot while walking. -0.7 to 1 m/s.
     */
    void forward(float distance, float velocity);

    /**
     * Adds an instruction to rotate in place.
     * @param theta Angle to rotate (in rad).
     * @param omega Angular velocity (in rad/s). -2pi/3 to 2pi/3 rad/s.
     */
    void rotate(float theta, float omega);    
};
