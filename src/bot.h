#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <functional>
using namespace UNITREE_LEGGED_SDK;

/**
 * Output of an instruction.
 */
struct InstructionData {    
    /** Command to execute. */
    HighCmd cmd; 
    /** Whether this instruction is done. */
    bool done;
};

/** 
 * Lambda that takes in initial state and current state (in that order)
 * and outputs a command for the current timestamp and whether the action is done. 
 * @see Bot::RobotControl()
 */
using Instruction = std::function<InstructionData(HighState, HighState)>;

/**
 * Class representing the Unitree A1.
 */
class Bot
{
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    HighState initial_state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    std::vector<Instruction> instructions;
    int index = 0;
    bool executing = false;
    
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
     * @param v velocity Velocity of robot while walking.
     */
    void forward(float distance, float velocity);

    /**
     * Adds an instruction to rotate in place.
     * @param theta Angle to rotate (in radians).
     */
    void rotate(float theta);    
};
